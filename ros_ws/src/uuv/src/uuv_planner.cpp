#include <chrono>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <algorithm>
#include <unordered_map>
#include <sstream>
#include <cmath>
#include <functional>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

// Custom messages (note: lowercase headers on Linux)
#include "custom_msg/msg/plan.hpp"
#include "custom_msg/msg/event.hpp"

using namespace std::chrono_literals;

// roles and states
enum class Role     { SUB1, SUB2, MID_TIER };
enum class MidState { SURFACE_INIT, HANDOFF_1, HANDOFF_2, SURFACE_RELAY, RETURN_END };
enum class SubState { HOLD_A,  FOLLOW_A,     FOLLOW_B,   PATROL_C,      HOLD_B    };

// helpers
static geometry_msgs::msg::Point XYZ(double x,double y,double z){ geometry_msgs::msg::Point p; p.x=x;p.y=y;p.z=z; return p; }
static double now_unix() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
static builtin_interfaces::msg::Time toStamp(double t){
  builtin_interfaces::msg::Time st;
  if (t < 0) { st.sec = 0; st.nanosec = 0; return st; }
  const auto sec = static_cast<int32_t>(std::floor(t));
  const double frac = t - static_cast<double>(sec);
  st.sec = sec;
  st.nanosec = static_cast<uint32_t>(std::round(frac * 1e9));
  return st;
}
static double fromStamp(const builtin_interfaces::msg::Time& st){
  return static_cast<double>(st.sec) + static_cast<double>(st.nanosec) * 1e-9;
}

// plans + structures
struct Waypoint { double t; double x,y,z; };   // t = unix seconds
using PlanVec = std::vector<Waypoint>;

// Convert between internal PlanVec and custom_msg/Plan
// NOTE: format unchanged: caller still passes the frame tag they used before ("mid", "relay", etc.)
static custom_msg::msg::Plan toPlanMsg(const PlanVec& plan, const std::string& frame)
{
  custom_msg::msg::Plan out;
  out.header.frame_id = frame;
  out.header.stamp = toStamp(now_unix());

  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = out.header.stamp;

  path.poses.reserve(plan.size());
  for(const auto& w: plan){
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.header.stamp = toStamp(w.t);
    ps.pose.position.x = w.x;
    ps.pose.position.y = w.y;
    ps.pose.position.z = w.z;
    ps.pose.orientation.w = 1.0; // identity orientation; not used in planning
    path.poses.push_back(std::move(ps));
  }
  out.paths.push_back(std::move(path));
  return out;
}

static PlanVec fromPlanMsg(const custom_msg::msg::Plan& msg)
{
  PlanVec plan;
  for(const auto& path : msg.paths){
    for(const auto& ps : path.poses){
      plan.push_back(Waypoint{
        fromStamp(ps.header.stamp),
        ps.pose.position.x,
        ps.pose.position.y,
        ps.pose.position.z
      });
    }
  }
  std::sort(plan.begin(), plan.end(), [](auto&a,auto&b){return a.t<b.t;});
  return plan;
}

// merge add into base; return number of newly added points (exact match on t,x,y,z)
static std::size_t mergeIntoCount(PlanVec& base, const PlanVec& add){
  std::size_t newc = 0;
  for(const auto& w: add){
    bool has=false;
    for(const auto& v: base){
      if (std::fabs(v.t-w.t)<1e-9 && std::fabs(v.x-w.x)<1e-9 && std::fabs(v.y-w.y)<1e-9 && std::fabs(v.z-w.z)<1e-9){ has=true; break; }
    }
    if(!has){ base.push_back(w); ++newc; }
  }
  std::sort(base.begin(), base.end(), [](auto&a,auto&b){return a.t<b.t;});
  return newc;
}

// sanitize: strictly-increasing timestamps (no duplicates) and drop huge jumps
static void sanitizePlanHistory(PlanVec& v, double max_jump_m = 30.0){
  if (v.empty()) return;
  std::sort(v.begin(), v.end(), [](const Waypoint& a, const Waypoint& b){ return a.t < b.t; });
  PlanVec out; out.reserve(v.size());
  const double eps = 1e-6; // 1 microsecond
  double last_t = -1e300;
  bool have_prev = false;
  Waypoint prev{};
  for (auto w : v){
    if (w.t <= last_t) w.t = last_t + eps; // make strictly increasing
    bool ok = true;
    if (have_prev){
      const double dx = w.x - prev.x;
      const double dy = w.y - prev.y;
      const double dz = w.z - prev.z;
      const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dist > max_jump_m) ok = false; // drop “teleport”
    }
    if (ok){
      out.push_back(w);
      prev = w;
      last_t = w.t;
      have_prev = true;
    }
  }
  v.swap(out);
}

// linear interpolation of expected pose at time t; fallback to nearest
static bool expectedPoseAt(const PlanVec& plan, double t, geometry_msgs::msg::Point& out){
  if (plan.empty()) return false;
  if (t <= plan.front().t){ out = XYZ(plan.front().x, plan.front().y, plan.front().z); return true; }
  if (t >= plan.back().t) { out = XYZ(plan.back().x,  plan.back().y,  plan.back().z);  return true; }
  for (std::size_t i=1;i<plan.size();++i){
    if (t <= plan[i].t){
      const auto& a = plan[i-1]; const auto& b = plan[i];
      const double denom = std::max(1e-9, (b.t - a.t));
      const double u = (t - a.t) / denom;
      out = XYZ(a.x + u*(b.x-a.x), a.y + u*(b.y-a.y), a.z + u*(b.z-a.z));
      return true;
    }
  }
  return false;
}

class SubNode : public rclcpp::Node {
public:
  explicit SubNode(const std::string& sub_id)
  : Node(sub_id + "_node"), sub_id_(sub_id)
  {
    // parameters
    this->declare_parameter<std::vector<std::string>>("team_ids",   {"subMID","subA","subB"});
    this->declare_parameter<std::string>("leader_id", "subMID");
    this->declare_parameter<double>("start_epoch",    0.0);
    this->declare_parameter<double>("state_dwell_s",  20.0);
    this->declare_parameter<bool>  ("publish_current_pose", false);
    this->declare_parameter<int64_t>("Identifier", -1);
    // Optional: identity numbers aligned with team_ids (stable across role rotation)
    this->declare_parameter<std::vector<int64_t>>("team_identity_numbers", {0,1,2});

    Identifier = this->get_parameter("Identifier").as_int();  // store my stable Identifier number
    team_ids_            = this->get_parameter("team_ids").as_string_array();
    leader_id_           = this->get_parameter("leader_id").as_string();
    start_epoch_         = this->get_parameter("start_epoch").as_double();
    dwell_s_             = this->get_parameter("state_dwell_s").as_double();
    publish_current_pose_= this->get_parameter("publish_current_pose").as_bool();
    team_identity_numbers_ = this->get_parameter("team_identity_numbers").as_integer_array();

    // Ensure presence + dedupe of my sub_id in team_ids
    if (std::find(team_ids_.begin(), team_ids_.end(), sub_id_) == team_ids_.end())
      team_ids_.push_back(sub_id_);
    { // dedupe preserving first occurrence
      std::vector<std::string> uniq; uniq.reserve(team_ids_.size());
      for (auto& s: team_ids_) if (std::find(uniq.begin(), uniq.end(), s)==uniq.end()) uniq.push_back(s);
      team_ids_.swap(uniq);
    }

    // Align identity_numbers vector length to team_ids
    if (team_identity_numbers_.size() != team_ids_.size()){
      // fallback: identity = index
      team_identity_numbers_.resize(team_ids_.size());
      for (size_t i=0;i<team_ids_.size();++i) team_identity_numbers_[i] = static_cast<int64_t>(i);
    }

    // Build mapping team_id -> identity number
    for (size_t i=0;i<team_ids_.size(); ++i){
      idnum_by_teamid_[team_ids_[i]] = team_identity_numbers_[i];
    }

    auto it = std::find(team_ids_.begin(), team_ids_.end(), leader_id_);
    if (it == team_ids_.end()) {
      RCLCPP_WARN(get_logger(), "leader_id '%s' not in team_ids; using team_ids[0]", leader_id_.c_str());
      leader0_idx_ = 0; leader_id_ = team_ids_.front();
    } else {
      leader0_idx_ = static_cast<int>(std::distance(team_ids_.begin(), it));
    }

    if (start_epoch_ <= 0.0) {
      start_epoch_ = std::ceil(now_unix()) + 2.0;
      RCLCPP_INFO(get_logger(), "[%s] start_epoch not set; using %.0f", sub_id_.c_str(), start_epoch_);
    }

    // Publishers
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/" + sub_id_ + "/target_pose", 10);
    if (publish_current_pose_) pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/" + sub_id_ + "/pose", 10);

    // Plan pubs (custom Plan) by ROLE topics (unchanged)
    plans_pub_sub1_ = create_publisher<custom_msg::msg::Plan>("/plans/sub1", 10);
    plans_pub_sub2_ = create_publisher<custom_msg::msg::Plan>("/plans/sub2", 10);
    plans_pub_mid_  = create_publisher<custom_msg::msg::Plan>("/plans/mid",  10);

    echo_pub_sub1_ = create_publisher<custom_msg::msg::Plan>("/echo/sub1", 10);
    echo_pub_sub2_ = create_publisher<custom_msg::msg::Plan>("/echo/sub2", 10);
    echo_pub_mid_  = create_publisher<custom_msg::msg::Plan>("/echo/mid",  10);

    // Relay pubs
    relay_pub_sub1_ = create_publisher<custom_msg::msg::Plan>("/relay/sub1", 10);
    relay_pub_sub2_ = create_publisher<custom_msg::msg::Plan>("/relay/sub2", 10);
    relay_pub_mid_  = create_publisher<custom_msg::msg::Plan>("/relay/mid",  10);
    relay_snapshot_json_pub_ = create_publisher<std_msgs::msg::String>("/relay/snapshot_json", 10);
    relay_events_json_pub_   = create_publisher<std_msgs::msg::String>("/relay/events_json",   10);

    // Expected pose pubs
    exp_pub_sub1_ = create_publisher<geometry_msgs::msg::PoseStamped>("/expected/sub1/pose", 10);
    exp_pub_sub2_ = create_publisher<geometry_msgs::msg::PoseStamped>("/expected/sub2/pose", 10);
    exp_pub_mid_  = create_publisher<geometry_msgs::msg::PoseStamped>("/expected/mid/pose",  10);

    // Event pubs (per-role topics)
    event_pub_sub1_ = create_publisher<custom_msg::msg::Event>("/events/sub1", 10);
    event_pub_sub2_ = create_publisher<custom_msg::msg::Event>("/events/sub2", 10);
    event_pub_mid_  = create_publisher<custom_msg::msg::Event>("/events/mid",  10);

    // subscriptions: Plans (custom Plan) by ROLE topics
    plans_sub_s1_ = create_subscription<custom_msg::msg::Plan>(
      "/plans/sub1", 10,
      [this](custom_msg::msg::Plan::ConstSharedPtr msg){
        this->onPlanTopic(*msg, "sub1");
      });

    plans_sub_s2_ = create_subscription<custom_msg::msg::Plan>(
      "/plans/sub2", 10,
      [this](custom_msg::msg::Plan::ConstSharedPtr msg){
        this->onPlanTopic(*msg, "sub2");
      });

    plans_sub_mid_ = create_subscription<custom_msg::msg::Plan>(
      "/plans/mid", 10,
      [this](custom_msg::msg::Plan::ConstSharedPtr msg){
        this->onPlanTopic(*msg, "mid");
      });

    // Echo (Plan) listeners
    echo_sub_s1_ = create_subscription<custom_msg::msg::Plan>(
      "/echo/sub1", 10,
      [this](custom_msg::msg::Plan::ConstSharedPtr msg){
        this->onPeerEcho(*msg, "sub1");
      });

    echo_sub_s2_ = create_subscription<custom_msg::msg::Plan>(
      "/echo/sub2", 10,
      [this](custom_msg::msg::Plan::ConstSharedPtr msg){
        this->onPeerEcho(*msg, "sub2");
      });

    echo_sub_mid_ = create_subscription<custom_msg::msg::Plan>(
      "/echo/mid", 10,
      [this](custom_msg::msg::Plan::ConstSharedPtr msg){
        this->onPeerEcho(*msg, "mid");
      });

    // Events for each role
    event_sub_s1_ = create_subscription<custom_msg::msg::Event>(
      "/events/sub1", 10,
      [this](custom_msg::msg::Event::ConstSharedPtr msg){
        this->onEvent(*msg, "sub1");
      });

    event_sub_s2_ = create_subscription<custom_msg::msg::Event>(
      "/events/sub2", 10,
      [this](custom_msg::msg::Event::ConstSharedPtr msg){
        this->onEvent(*msg, "sub2");
      });

    event_sub_mid_ = create_subscription<custom_msg::msg::Event>(
      "/events/mid", 10,
      [this](custom_msg::msg::Event::ConstSharedPtr msg){
        this->onEvent(*msg, "mid");
      });

    timer_ = create_wall_timer(std::chrono::duration<double>(0.25), std::bind(&SubNode::tick, this));

    RCLCPP_INFO(get_logger(),
      "[%s] ring=%zu leader0=%s(idx=%d) epoch=%.0f dwell=%.1fs",
      sub_id_.c_str(), team_ids_.size(), leader_id_.c_str(), leader0_idx_, start_epoch_, dwell_s_);
    RCLCPP_INFO(get_logger(),
      "[%s] id=%ld ring=%zu leader0=%s(idx=%d) epoch=%.0f dwell=%.1fs",
      sub_id_.c_str(), static_cast<long>(Identifier),
      team_ids_.size(), leader_id_.c_str(), leader0_idx_, start_epoch_, dwell_s_);

  }

private:
  int64_t Identifier{-1};
  std::vector<int64_t> team_identity_numbers_;                 // aligned to team_ids_
  std::unordered_map<std::string,int64_t> idnum_by_teamid_;    // team_id -> identity number

  struct Sched {
    Role my_role;
    MidState mid_state;    // valid if my_role==MID_TIER
    SubState sub_state;    // valid if my_role!=MID_TIER
    std::string id_mid, id_s1, id_s2; // team_ids of role holders at time t
    int slot5;
  };

  // Compute schedule *at arbitrary time* (used for attribution by waypoint time)
  Sched compute(double tunix) const {
    const int N = static_cast<int>(team_ids_.size());
    const double T = 5.0 * dwell_s_;
    const double dt = std::max(0.0, tunix - start_epoch_);
    const long rot_k = (T>0.0)? static_cast<long>(std::floor(dt / T)) : 0L;

    const int leader_idx = (leader0_idx_ + static_cast<int>(rot_k % N) + N) % N;
    const int s1_idx = (leader_idx + 1) % N;
    const int s2_idx = (leader_idx + 2) % N;

    const std::string& mid = team_ids_[leader_idx];
    const std::string& s1  = team_ids_[s1_idx];
    const std::string& s2  = team_ids_[s2_idx];

    const long slot = (dwell_s_>0.0)? static_cast<long>(std::floor(std::fmod(dt, T) / dwell_s_)) : 0L;
    const int slot5 = static_cast<int>((slot % 5 + 5) % 5);

    MidState ms = MidState::SURFACE_INIT;
    if      (slot5==1) ms=MidState::HANDOFF_1;
    else if (slot5==2) ms=MidState::HANDOFF_2;
    else if (slot5==3) ms=MidState::SURFACE_RELAY;
    else if (slot5==4) ms=MidState::RETURN_END;

    SubState ss = SubState::HOLD_A;
    if      (slot5==1) ss=SubState::FOLLOW_A;
    else if (slot5==2) ss=SubState::FOLLOW_B;
    else if (slot5==3) ss=SubState::PATROL_C;
    else if (slot5==4) ss=SubState::HOLD_B;

    Role r=Role::SUB1;
    const double tnow = tunix;
    if (sub_id_==mid) r=Role::MID_TIER;
    else if (sub_id_==s1) r=Role::SUB1;
    else if (sub_id_==s2) r=Role::SUB2;

    (void)tnow; // r depends on real-time identity, but attribution uses id_* fields below
    return Sched{r,ms,ss,mid,s1,s2,slot5};
  }

  // lookup helpers
  std::string roleKeyNow() const {
    const auto sch = compute(now_unix());
    if (sch.my_role == Role::MID_TIER) return "mid";
    if (sch.my_role == Role::SUB1)     return "sub1";
    return "sub2";
  }
  static std::string roleOfTeamId(const std::string& team_id, const Sched& sch){
    if (team_id == sch.id_mid) return "mid";
    if (team_id == sch.id_s1)  return "sub1";
    if (team_id == sch.id_s2)  return "sub2";
    return "none";
  }
  int64_t identityOfTeamId(const std::string& team_id) const {
    auto it = idnum_by_teamid_.find(team_id);
    if (it != idnum_by_teamid_.end()) return it->second;
    return -1;
  }

  geometry_msgs::msg::Point midTarget(MidState s) const {
    switch(s){
      case MidState::SURFACE_INIT: return XYZ(  0,  0,  -2);
      case MidState::HANDOFF_1:    return XYZ( 50, 20, -10);
      case MidState::HANDOFF_2:    return XYZ( 50, 60, -10);
      case MidState::SURFACE_RELAY:return XYZ(  0,  5,  -2);
      case MidState::RETURN_END:   return XYZ(100, 40, -10);
    }
    return XYZ(0,0,-5);
  }
  geometry_msgs::msg::Point sub1Target(SubState s) const {
    switch(s){
      case SubState::HOLD_A:   return XYZ( 20, 20, -20);
      case SubState::FOLLOW_A: return XYZ( 35, 20, -20);
      case SubState::FOLLOW_B: return XYZ( 35, 60, -20);
      case SubState::PATROL_C: return XYZ( 10, 35, -22);
      case SubState::HOLD_B:   return XYZ( 20, 40, -20);
    }
    return XYZ(0,0,-20);
  }
  geometry_msgs::msg::Point sub2Target(SubState s) const {
    switch(s){
      case SubState::HOLD_A:   return XYZ( 20, 60, -20);
      case SubState::FOLLOW_A: return XYZ( 35, 20, -20);
      case SubState::FOLLOW_B: return XYZ( 35, 60, -20);
      case SubState::PATROL_C: return XYZ( 10, 45, -22);
      case SubState::HOLD_B:   return XYZ( 20, 40, -20);
    }
    return XYZ(0,0,-20);
  }

  // publish my event at SURFACE_INIT (non-MID roles)
  void publishMyEventIfNeeded(const Sched& sch)
  {
    if (sch.my_role == Role::MID_TIER) return;

    custom_msg::msg::Event ev;
    ev.header.stamp = toStamp(now_unix());
    ev.header.frame_id = sub_id_;
    ev.waypoints.clear();

    geometry_msgs::msg::PoseStamped ps;
    ps.header = ev.header;
    ps.pose.position = (sch.my_role==Role::SUB1) ? sub1Target(sch.sub_state) : sub2Target(sch.sub_state);
    ps.pose.orientation.w = 1.0;
    ev.waypoints.push_back(ps);

    ev.event_type = last_local_event_type_.empty() ? "nothing_new" : last_local_event_type_;

    if (sch.my_role==Role::SUB1) event_pub_sub1_->publish(ev);
    else                         event_pub_sub2_->publish(ev);

    RCLCPP_INFO(get_logger(), "[%s] Published SURFACE_INIT event on /events/%s: %s",
      sub_id_.c_str(),
      (sch.my_role==Role::SUB1?"sub1":"sub2"),
      ev.event_type.c_str());
  }

  void onEvent(const custom_msg::msg::Event& ev, const std::string& role_key){
    latest_events_[role_key] = ev;
    ++events_since_surface_[role_key];
    RCLCPP_INFO(get_logger(), "[%s] Received Event from %s: %s", sub_id_.c_str(), role_key.c_str(), ev.event_type.c_str());
  }

  // make and publish events JSON uplink at SURFACE_INIT if it is the mid node
  void uplinkEventsJSON(const Sched& sch){
    const double jstime = now_unix();
    std::ostringstream js;
    js << "{"
      << "\"kind\":\"surface_relay_snapshot\","
      << "\"sender\":\"" << sub_id_ << "_node\","
      << "\"Identifier\":" << Identifier << ","
      << "\"role\":\"MID_TIER\","
      << "\"cycle\":{"
      <<   "\"epoch\":" << std::fixed << start_epoch_ << ","
      <<   "\"dwell_s\":" << dwell_s_ << ","
      <<   "\"slot\":" << sch.slot5 << "},"
      << "\"events\":{";

    auto writeEv = [&](const std::string& key){
      js << "\"" << key << "\":{";
      if (latest_events_.count(key)){
        const auto& ev = latest_events_[key];
        double x=0,y=0,z=0;
        if (!ev.waypoints.empty()){
          const auto& p = ev.waypoints.front().pose.position;
          x=p.x; y=p.y; z=p.z;
        }
        js << "\"event_type\":\"" << ev.event_type << "\","
           << "\"x\":" << x << ",\"y\":" << y << ",\"z\":" << z << "}";
      } else {
        js << "\"event_type\":\"nothing_new\",\"x\":0,\"y\":0,\"z\":0}";
      }
    };

    writeEv("mid");   js << ",";
    writeEv("sub1");  js << ",";
    writeEv("sub2");

    js << "},"
       << "\"new_counts\":{"
       <<   "\"mid\":"  << events_since_surface_["mid"]  << ","
       <<   "\"sub1\":" << events_since_surface_["sub1"] << ","
       <<   "\"sub2\":" << events_since_surface_["sub2"] << "},"
       << "\"sent_at\":" << std::fixed << jstime
       << "}";

    std_msgs::msg::String out;
    out.data = js.str();
    relay_events_json_pub_->publish(out);

    events_since_surface_["mid"]  = 0;
    events_since_surface_["sub1"] = 0;
    events_since_surface_["sub2"] = 0;

    RCLCPP_INFO(get_logger(), "[%s] Uplinked events JSON at SURFACE_INIT", sub_id_.c_str());
  }

  // plans per tick, state cycles
  void tick(){
    const double t = now_unix();
    const auto sch = compute(t);

    if (sch.slot5 != last_slot5_ || sch.id_mid != last_mid_id_) {
      onSlotEnter(sch);
      last_slot5_ = sch.slot5;
      last_mid_id_ = sch.id_mid;
    }

    // Publish nav target per role/state
    geometry_msgs::msg::PoseStamped tgt;
    tgt.header.stamp = this->get_clock()->now();
    tgt.header.frame_id = sub_id_;
    if (sch.my_role == Role::MID_TIER) {
      tgt.pose.position = midTarget(sch.mid_state);
    } else if (sch.my_role == Role::SUB1) {
      tgt.pose.position = sub1Target(sch.sub_state);
    } else {
      tgt.pose.position = sub2Target(sch.sub_state);
    }
    target_pub_->publish(tgt);
    if (publish_current_pose_) { if (pose_pub_) pose_pub_->publish(tgt); }

    publishExpectedPoses(t);

    if (static_cast<int>(t) % 2 == 0 && (last_log_second_ != static_cast<int>(t))) {
      last_log_second_ = static_cast<int>(t);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] role=%s mid=%s s1=%s s2=%s slot=%d dwell=%.1f (added: s1=%zu s2=%zu mid=%zu)",
        sub_id_.c_str(),
        (sch.my_role==Role::MID_TIER?"MID":(sch.my_role==Role::SUB1?"SUB1":"SUB2")),
        sch.id_mid.c_str(), sch.id_s1.c_str(), sch.id_s2.c_str(),
        sch.slot5, dwell_s_,
        added_since_surface_sub1_, added_since_surface_sub2_, added_since_surface_mid_);
    }
  }

  // Called locally when we enter a new 0..4 slot
  void onSlotEnter(const Sched& sch){
    published_this_slot_ = false;

    // At the start of each SURFACE_INIT, non-MID subs publish their current event
    if (sch.mid_state == MidState::SURFACE_INIT) {
      publishMyEventIfNeeded(sch);
    }

    if (sch.my_role == Role::MID_TIER) {
      switch (sch.mid_state) {
        case MidState::SURFACE_INIT:
          RCLCPP_INFO(get_logger(),
            "[%s] MID SURFACE_INIT: surfacing — uploading known plans (s1=%zu, s2=%zu, mid=%zu); requesting/latest events",
            sub_id_.c_str(), known_sub1_.size(), known_sub2_.size(), known_mid_.size());
          ensureSeedPlans();
          resetAddedSinceSurface();
          uplinkEventsJSON(sch);
          break;

        case MidState::HANDOFF_1:
          midHandoffOnce("sub1", sch.id_s1);
          break;

        case MidState::HANDOFF_2:
          midHandoffOnce("sub2", sch.id_s2);
          break;

        case MidState::SURFACE_RELAY:
          RCLCPP_INFO(get_logger(),
            "[%s] MID SURFACE_RELAY: relaying merged knowledge (new since last surface: s1=%zu, s2=%zu, mid=%zu)",
            sub_id_.c_str(), added_since_surface_sub1_, added_since_surface_sub2_, added_since_surface_mid_);
          surfaceRelayAggregate();
          resetAddedSinceSurface();
          break;

        case MidState::RETURN_END:
          RCLCPP_INFO(get_logger(), "[%s] MID RETURN_END: navigating to end-of-line / prep role rotation", sub_id_.c_str());
          break;
      }
    }
  }

  // MID: one-shot handoff publisher in the slot (format unchanged)
  void midHandoffOnce(const std::string& role_key, const std::string& recipient_id){
    if (published_this_slot_) return;
    published_this_slot_ = true;

    RCLCPP_INFO(get_logger(),
      "[%s] MID handoff → %s (role=%s): publishing /plans/%s and /plans/mid; echoing /echo/mid",
      sub_id_.c_str(), recipient_id.c_str(), role_key.c_str(), role_key.c_str());

    if (role_key == "sub1") {
      plans_pub_sub1_->publish(toPlanMsg(known_sub1_, "mid"));
    } else {
      plans_pub_sub2_->publish(toPlanMsg(known_sub2_, "mid"));
    }
    plans_pub_mid_->publish (toPlanMsg(known_mid_,  "mid"));

    if (!known_mid_prev_.empty()) {
      auto pa = toPlanMsg(known_mid_prev_, "mid_prev");
      echo_pub_mid_->publish(pa);
    }
    known_mid_prev_ = known_mid_;
  }

  // Split a plan by identity based on each waypoint's timestamp and the role topic it arrived on.
  // This preserves on-wire format but yields identity-accurate history.
  std::unordered_map<int64_t, PlanVec>
  splitByIdentityFromRole(const PlanVec& p, const std::string& role_key) const {
    std::unordered_map<int64_t, PlanVec> out;
    for (const auto& w : p){
      const auto sch_w = compute(w.t); // role holders at this waypoint time
      std::string team_id_holder =
        (role_key=="mid") ? sch_w.id_mid : (role_key=="sub1" ? sch_w.id_s1 : sch_w.id_s2);
      int64_t idn = identityOfTeamId(team_id_holder);
      if (idn >= 0) out[idn].push_back(w);
    }
    return out;
  }

  // ROLE-topic plan reception → merge role knowledge AND per-identity store (by waypoint time)
  void onPlanTopic(const custom_msg::msg::Plan& pm, const std::string& topic_key /* "sub1"|"sub2"|"mid" */) {
    const std::string my_key = roleKeyNow();
    PlanVec new_plan = fromPlanMsg(pm);

    // Merge into role-based knowledge (kept for expected poses + legacy pubs)
    std::size_t added = 0;
    if (topic_key=="sub1")      added = mergeIntoCount(known_sub1_, new_plan), added_since_surface_sub1_ += added;
    else if (topic_key=="sub2") added = mergeIntoCount(known_sub2_, new_plan), added_since_surface_sub2_ += added;
    else                        added = mergeIntoCount(known_mid_,  new_plan), added_since_surface_mid_  += added;

    // Also merge into the PER-IDENTITY store using per-waypoint attribution
    auto chunks = splitByIdentityFromRole(new_plan, topic_key);
    for (auto& kv : chunks){
      auto& vec = per_identity_plans_[kv.first]; // creates if missing
      mergeIntoCount(vec, kv.second);
    }

    // If this plan is for me-in-my-current-role, do echo bookkeeping
    if (topic_key == my_key) {
      if (!my_prev_plan_.empty()) {
        auto echo_pa = toPlanMsg(my_prev_plan_, my_key + std::string("_prev"));
        echo_pub_for_key(my_key)->publish(echo_pa);
        RCLCPP_INFO(get_logger(), "[%s] ECHO %s prev plan (%zu pts)", sub_id_.c_str(), my_key.c_str(), my_prev_plan_.size());
      }
      my_prev_plan_ = my_plan_;
      my_plan_ = std::move(new_plan);
      RCLCPP_INFO(get_logger(), "[%s] MY PLAN updated for %s (%zu pts, +%zu new merged)", sub_id_.c_str(), my_key.c_str(), my_plan_.size(), added);
    }
  }

  // Any node: on peer echo, merge into knowledge & counters (role-based only)
  void onPeerEcho(const custom_msg::msg::Plan& pm, const std::string& who){
    PlanVec p = fromPlanMsg(pm);
    std::size_t added = 0;
    if (who=="sub1")      added = mergeIntoCount(known_sub1_, p), added_since_surface_sub1_ += added;
    else if (who=="sub2") added = mergeIntoCount(known_sub2_, p), added_since_surface_sub2_ += added;
    else if (who=="mid")  added = mergeIntoCount(known_mid_,  p), added_since_surface_mid_  += added;
    RCLCPP_INFO(get_logger(), "[%s] Merged peer echo from %s (+%zu new pts)", sub_id_.c_str(), who.c_str(), added);
  }

  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr& echo_pub_for_key(const std::string& k){
    if (k=="sub1") return echo_pub_sub1_;
    if (k=="sub2") return echo_pub_sub2_;
    return echo_pub_mid_;
  }

  // JSON helper to serialize a plan/history array
  static std::string planToJsonArray(const PlanVec& p){
    std::ostringstream os; os << "[";
    for (size_t i=0;i<p.size();++i){
      const auto& w=p[i];
      os << "{\"t\":"<<std::fixed<<w.t<<",\"x\":"<<w.x<<",\"y\":"<<w.y<<",\"z\":"<<w.z<<"}";
      if(i+1<p.size()) os << ",";
    }
    os << "]";
    return os.str();
  }

  // SURFACE_RELAY: publish merged knowledge + JSON snapshot keyed by Identity
  void surfaceRelayAggregate(){
    // publish role-based merged knowledge (unchanged)
    relay_pub_sub1_->publish(toPlanMsg(known_sub1_, "relay"));
    relay_pub_sub2_->publish(toPlanMsg(known_sub2_, "relay"));
    relay_pub_mid_->publish (toPlanMsg(known_mid_,  "relay"));

    const auto sch = compute(now_unix());

    // Ensure we have entries for all team members
    for (const auto& team_id : team_ids_){
      int64_t idn = identityOfTeamId(team_id);
      (void) per_identity_plans_[idn]; // ensure default-created
    }

    // Create sanitized copies, and (optionally) cap extreme jumps
    std::unordered_map<int64_t, PlanVec> plans_sanitized;
    for (const auto& team_id : team_ids_){
      const int64_t idn = identityOfTeamId(team_id);
      PlanVec cp = per_identity_plans_[idn];     // copy
      sanitizePlanHistory(cp, 30.0);
      plans_sanitized[idn]   = cp;               // history == plans
    }

    // Also still provide the role-based arrays for backward compatibility
    PlanVec k_mid = known_mid_, k_s1 = known_sub1_, k_s2 = known_sub2_;
    sanitizePlanHistory(k_mid, 30.0);
    sanitizePlanHistory(k_s1,  30.0);
    sanitizePlanHistory(k_s2,  30.0);

    // Build JSON — FORMAT UNCHANGED vs your last version
    std::ostringstream js;
    js << "{"
       << "\"kind\":\"surface_relay_snapshot\","
       << "\"sender\":\"" << sub_id_ << "_node\","
       << "\"Identifier\":" << Identifier << ","
       << "\"role\":\"MID_TIER\","
       << "\"cycle\":{"
       <<   "\"epoch\":" << std::fixed << start_epoch_ << ","
       <<   "\"dwell_s\":" << dwell_s_ << ","
       <<   "\"slot\":" << sch.slot5 << ","
       <<   "\"ids\":{"
       <<     "\"mid\":\""  << sch.id_mid << "\","
       <<     "\"sub1\":\"" << sch.id_s1  << "\","
       <<     "\"sub2\":\"" << sch.id_s2  << "\"}"
       << "},"
       << "\"stats\":{"
       <<   "\"new_since_last_surface\":{"
       <<     "\"sub1\":" << added_since_surface_sub1_ << ","
       <<     "\"sub2\":" << added_since_surface_sub2_ << ","
       <<     "\"mid\":"  << added_since_surface_mid_  << "}"
       << "},"
       << "\"plans_by_role\":{"
       <<   "\"mid\":"  << planToJsonArray(k_mid) << ","
       <<   "\"sub1\":" << planToJsonArray(k_s1)  << ","
       <<   "\"sub2\":" << planToJsonArray(k_s2)  << "},"
       << "\"by_identity\":{";

    // by_identity: "IDNUM": { "team_id": "...", "role_now":"...", "plan":[...], "history":[...] }
    for (size_t i=0;i<team_ids_.size(); ++i){
      const std::string& team_id = team_ids_[i];
      const int64_t idn = team_identity_numbers_[i];
      const std::string role_now = roleOfTeamId(team_id, sch);
      const PlanVec& p  = plans_sanitized[idn];
      js << "\"" << idn << "\":{"
         <<   "\"team_id\":\"" << team_id << "\","
         <<   "\"role_now\":\"" << role_now << "\","
         <<   "\"plan\":"    << planToJsonArray(p) << ","
         <<   "\"history\":" << planToJsonArray(p) << "}";
      if (i + 1 < team_ids_.size()) js << ",";
    }
    js << "},"
       << "\"sent_at\":" << std::fixed << now_unix()
       << "}";

    std_msgs::msg::String out;
    out.data = js.str();
    relay_snapshot_json_pub_->publish(out);

    RCLCPP_INFO(get_logger(),
      "[%s] SURFACE_RELAY | relayed merged plans (by role + by identity) | new since last surface: sub1=%zu sub2=%zu mid=%zu",
      sub_id_.c_str(), added_since_surface_sub1_, added_since_surface_sub2_, added_since_surface_mid_);
  }

  void resetAddedSinceSurface(){
    added_since_surface_sub1_ = 0;
    added_since_surface_sub2_ = 0;
    added_since_surface_mid_  = 0;
  }

  // Expected poses publisher (by role)
  void publishExpectedPoses(double t){
    geometry_msgs::msg::PoseStamped ps; ps.header.stamp = this->get_clock()->now();
    geometry_msgs::msg::Point pt;
    if (expectedPoseAt(known_sub1_, t, pt)) { ps.header.frame_id="sub1"; ps.pose.position=pt; exp_pub_sub1_->publish(ps); }
    if (expectedPoseAt(known_sub2_, t, pt)) { ps.header.frame_id="sub2"; ps.pose.position=pt; exp_pub_sub2_->publish(ps); }
    if (expectedPoseAt(known_mid_,  t, pt)) { ps.header.frame_id="mid";  ps.pose.position=pt; exp_pub_mid_->publish(ps);  }
  }

  // Seed some plans so handoffs have content (also seed per-identity from current role holders)
  void ensureSeedPlans(){
    if (known_mid_.empty()){
      known_mid_ = {
        {start_epoch_ +  1*dwell_s_, 50,20,-10},
        {start_epoch_ +  2*dwell_s_, 50,60,-10},
        {start_epoch_ +  6*dwell_s_, 50,20,-10},
        {start_epoch_ +  7*dwell_s_, 50,60,-10},
      };
    }
    if (known_sub1_.empty()){
      known_sub1_ = {
        {start_epoch_ +  1*dwell_s_, 35,20,-20},
        {start_epoch_ +  2*dwell_s_, 35,60,-20},
        {start_epoch_ +  6*dwell_s_, 35,20,-20},
        {start_epoch_ +  7*dwell_s_, 35,60,-20},
      };
    }
    if (known_sub2_.empty()){
      known_sub2_ = {
        {start_epoch_ +  1*dwell_s_, 35,20,-20},
        {start_epoch_ +  2*dwell_s_, 35,60,-20},
        {start_epoch_ +  6*dwell_s_, 35,20,-20},
        {start_epoch_ +  7*dwell_s_, 35,60,-20},
      };
    }

    // Mirror into per-identity for current holders (so identity snapshots are populated from the start)
    const auto sch = compute(now_unix());
    int64_t id_mid  = identityOfTeamId(sch.id_mid);
    int64_t id_sub1 = identityOfTeamId(sch.id_s1);
    int64_t id_sub2 = identityOfTeamId(sch.id_s2);
    if (id_mid  >= 0) mergeIntoCount(per_identity_plans_[id_mid],  known_mid_);
    if (id_sub1 >= 0) mergeIntoCount(per_identity_plans_[id_sub1], known_sub1_);
    if (id_sub2 >= 0) mergeIntoCount(per_identity_plans_[id_sub2], known_sub2_);
  }

  // state / last-event info
  std::string last_local_event_type_;  // e.g., "foreign_uuv", "pipeline_break", "low_battery"

  // state info
  std::string sub_id_;
  std::vector<std::string> team_ids_;
  std::string leader_id_;
  int    leader0_idx_{0};
  double start_epoch_{0.0};
  double dwell_s_{20.0};
  bool   publish_current_pose_{false};

  int  last_slot5_{-1};
  std::string last_mid_id_;
  bool published_this_slot_{false};

  // Plans (role-based, for expected poses and legacy pubs)
  PlanVec my_plan_;
  PlanVec my_prev_plan_;
  PlanVec known_sub1_;
  PlanVec known_sub2_;
  PlanVec known_mid_;
  PlanVec known_mid_prev_;

  // Per-identity plan/history store (history == plans; sanitized on serialization)
  std::unordered_map<int64_t, PlanVec> per_identity_plans_;

  // New plan points accumulated since last surface (for SURFACE_RELAY reporting)
  std::size_t added_since_surface_sub1_{0};
  std::size_t added_since_surface_sub2_{0};
  std::size_t added_since_surface_mid_{0};

  // Latest events per role + counters since last surface
  std::unordered_map<std::string, custom_msg::msg::Event> latest_events_;
  std::unordered_map<std::string, std::size_t> events_since_surface_{{"mid",0},{"sub1",0},{"sub2",0}};

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr plans_pub_sub1_;
  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr plans_pub_sub2_;
  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr plans_pub_mid_;

  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr echo_pub_sub1_;
  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr echo_pub_sub2_;
  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr echo_pub_mid_;

  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr relay_pub_sub1_;
  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr relay_pub_sub2_;
  rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr relay_pub_mid_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr relay_snapshot_json_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr relay_events_json_pub_;

  rclcpp::Publisher<custom_msg::msg::Event>::SharedPtr event_pub_sub1_;
  rclcpp::Publisher<custom_msg::msg::Event>::SharedPtr event_pub_sub2_;
  rclcpp::Publisher<custom_msg::msg::Event>::SharedPtr event_pub_mid_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exp_pub_sub1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exp_pub_sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exp_pub_mid_;

  // Subscribers
  rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr plans_sub_s1_;
  rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr plans_sub_s2_;
  rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr plans_sub_mid_;
  rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr echo_sub_s1_;
  rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr echo_sub_s2_;
  rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr echo_sub_mid_;

  rclcpp::Subscription<custom_msg::msg::Event>::SharedPtr event_sub_s1_;
  rclcpp::Subscription<custom_msg::msg::Event>::SharedPtr event_sub_s2_;
  rclcpp::Subscription<custom_msg::msg::Event>::SharedPtr event_sub_mid_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  int last_log_second_{-1};
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto boot = std::make_shared<rclcpp::Node>("bootstrap");
  boot->declare_parameter<std::string>("sub_id");
  const std::string sub_id = boot->get_parameter("sub_id").as_string();
  boot.reset();

  auto node = std::make_shared<SubNode>(sub_id);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
