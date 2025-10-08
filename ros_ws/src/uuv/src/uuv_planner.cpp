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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

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

// plans + structures 
struct Waypoint { double t; double x,y,z; };   // t is optional unix time if provided
using Plan = std::vector<Waypoint>;

static geometry_msgs::msg::PoseArray toPoseArray(const Plan& plan, const std::string& frame){
  geometry_msgs::msg::PoseArray pa;
  pa.header.frame_id = frame;
  pa.poses.reserve(plan.size());
  for(const auto& w: plan){
    geometry_msgs::msg::Pose p;
    p.position.x=w.x; p.position.y=w.y; p.position.z=w.z;
    p.orientation.w = w.t; // store unix time in w (convention)
    pa.poses.push_back(p);
  }
  return pa;
}

static Plan fromPoseArray(const geometry_msgs::msg::PoseArray& pa){
  Plan plan; plan.reserve(pa.poses.size());
  for(const auto& p: pa.poses)
    plan.push_back(Waypoint{p.orientation.w, p.position.x, p.position.y, p.position.z});
  std::sort(plan.begin(), plan.end(), [](auto&a,auto&b){return a.t<b.t;});
  return plan;
}

// merge add into base; return number of newly added points
static std::size_t mergeIntoCount(Plan& base, const Plan& add){
  std::size_t newc = 0;
  for(const auto& w: add){
    bool has=false;
    for(const auto& v: base){
      if (std::fabs(v.t-w.t)<1e-6 && std::fabs(v.x-w.x)<1e-6 && std::fabs(v.y-w.y)<1e-6 && std::fabs(v.z-w.z)<1e-6){ has=true; break; }
    }
    if(!has){ base.push_back(w); ++newc; }
  }
  std::sort(base.begin(), base.end(), [](auto&a,auto&b){return a.t<b.t;});
  return newc;
}

// linear interpolation of expected pose at time t; fallback to nearest
static bool expectedPoseAt(const Plan& plan, double t, geometry_msgs::msg::Point& out){
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
    this->declare_parameter<std::vector<std::string>>("team_ids",   {"subA","subB","subC"});
    this->declare_parameter<std::string>("leader_id", "subA");       // leader at start_epoch
    this->declare_parameter<double>("start_epoch",    0.0);          // common unix seconds for all
    this->declare_parameter<double>("state_dwell_s",  20.0);         // default dwell per slot
    this->declare_parameter<bool>  ("publish_current_pose", false);

    team_ids_            = this->get_parameter("team_ids").as_string_array();
    leader_id_           = this->get_parameter("leader_id").as_string();
    start_epoch_         = this->get_parameter("start_epoch").as_double();
    dwell_s_             = this->get_parameter("state_dwell_s").as_double();
    publish_current_pose_= this->get_parameter("publish_current_pose").as_bool();

    // Keep provided order; ensure I'm present once; dedupe preserving first occurrences
    if (std::find(team_ids_.begin(), team_ids_.end(), sub_id_) == team_ids_.end())
      team_ids_.push_back(sub_id_);
    std::vector<std::string> uniq; uniq.reserve(team_ids_.size());
    for (auto& s: team_ids_) if (std::find(uniq.begin(), uniq.end(), s)==uniq.end()) uniq.push_back(s);
    team_ids_.swap(uniq);

    auto it = std::find(team_ids_.begin(), team_ids_.end(), leader_id_);
    if (it == team_ids_.end()) {
      RCLCPP_WARN(get_logger(), "leader_id '%s' not in team_ids; using team_ids[0]", leader_id_.c_str());
      leader0_idx_ = 0; leader_id_ = team_ids_.front();
    } else {
      leader0_idx_ = static_cast<int>(std::distance(team_ids_.begin(), it));
    }

    if (start_epoch_ <= 0.0) {
      start_epoch_ = std::ceil(now_unix()) + 2.0; // give a small sync window
      RCLCPP_INFO(get_logger(), "[%s] start_epoch not set; using %.0f", sub_id_.c_str(), start_epoch_);
    }

    // Publishers 
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/" + sub_id_ + "/target_pose", 10);
    if (publish_current_pose_) pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/" + sub_id_ + "/pose", 10);

    // Plan pubs (MID uses these)
    plans_pub_sub1_ = create_publisher<geometry_msgs::msg::PoseArray>("/plans/sub1", 10);
    plans_pub_sub2_ = create_publisher<geometry_msgs::msg::PoseArray>("/plans/sub2", 10);
    plans_pub_mid_  = create_publisher<geometry_msgs::msg::PoseArray>("/plans/mid",  10);

    // Echo pubs (publish based on current role, not node name)
    echo_pub_sub1_ = create_publisher<geometry_msgs::msg::PoseArray>("/echo/sub1", 10);
    echo_pub_sub2_ = create_publisher<geometry_msgs::msg::PoseArray>("/echo/sub2", 10);
    echo_pub_mid_  = create_publisher<geometry_msgs::msg::PoseArray>("/echo/mid",  10);

    // Relay pubs (surface relay to “satcom” or logger)
    relay_pub_sub1_ = create_publisher<geometry_msgs::msg::PoseArray>("/relay/sub1", 10);
    relay_pub_sub2_ = create_publisher<geometry_msgs::msg::PoseArray>("/relay/sub2", 10);
    relay_pub_mid_  = create_publisher<geometry_msgs::msg::PoseArray>("/relay/mid",  10);

    // Expected pose pubs (everyone publishes what they think others should be doing now)
    exp_pub_sub1_ = create_publisher<geometry_msgs::msg::PoseStamped>("/expected/sub1/pose", 10);
    exp_pub_sub2_ = create_publisher<geometry_msgs::msg::PoseStamped>("/expected/sub2/pose", 10);
    exp_pub_mid_  = create_publisher<geometry_msgs::msg::PoseStamped>("/expected/mid/pose",  10);

    // ---- Subscribers (lambda form; no std::bind extra args) ----
    plans_sub_s1_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/plans/sub1", 10,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg){
        this->onPlanTopic(*msg, "sub1");
      });

    plans_sub_s2_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/plans/sub2", 10,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg){
        this->onPlanTopic(*msg, "sub2");
      });

    plans_sub_mid_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/plans/mid", 10,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg){
        this->onPlanTopic(*msg, "mid");
      });

    echo_sub_s1_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/echo/sub1", 10,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg){
        this->onPeerEcho(*msg, "sub1");
      });

    echo_sub_s2_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/echo/sub2", 10,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg){
        this->onPeerEcho(*msg, "sub2");
      });

    echo_sub_mid_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/echo/mid", 10,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg){
        this->onPeerEcho(*msg, "mid");
      });

    timer_ = create_wall_timer(std::chrono::duration<double>(0.25), std::bind(&SubNode::tick, this));

    RCLCPP_INFO(get_logger(),
      "[%s] ring=%zu leader0=%s(idx=%d) epoch=%.0f dwell=%.1fs",
      sub_id_.c_str(), team_ids_.size(), leader_id_.c_str(), leader0_idx_, start_epoch_, dwell_s_);
  }

private:
  struct Sched {
    Role my_role;
    MidState mid_state;    // valid if my_role==MID_TIER
    SubState sub_state;    // valid if my_role!=MID_TIER
    std::string id_mid, id_s1, id_s2;
    int slot5;
  };
  Sched compute(double tunix) const {
    const int N = static_cast<int>(team_ids_.size());
    const double T = 5.0 * dwell_s_;            // 5-slot macro cycle
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
    if (sub_id_==mid) r=Role::MID_TIER;
    else if (sub_id_==s1) r=Role::SUB1;
    else if (sub_id_==s2) r=Role::SUB2;

    return Sched{r,ms,ss,mid,s1,s2,slot5};
  }

  // The canonical role key ("mid","sub1","sub2") 
  std::string roleKeyNow() const {
    const auto sch = compute(now_unix());
    if (sch.my_role == Role::MID_TIER) return "mid";
    if (sch.my_role == Role::SUB1)     return "sub1";
    return "sub2";
  }

  geometry_msgs::msg::Point midTarget(MidState s) const {
    switch(s){
      case MidState::SURFACE_INIT: return XYZ(  0,  0,  -2);  // initial surface to fetch plans/reset
      case MidState::HANDOFF_1:    return XYZ( 50, 20, -10);
      case MidState::HANDOFF_2:    return XYZ( 50, 60, -10);
      case MidState::SURFACE_RELAY:return XYZ(  0,  5,  -2);  // second surfacing to relay aggregated info
      case MidState::RETURN_END:   return XYZ(100, 40, -10);
    }
    return XYZ(0,0,-5);
  }
  geometry_msgs::msg::Point sub1Target(SubState s) const {
    switch(s){
      case SubState::HOLD_A:   return XYZ( 20, 20, -20);
      case SubState::FOLLOW_A: return XYZ( 35, 20, -20);
      case SubState::FOLLOW_B: return XYZ( 35, 60, -20);
      case SubState::PATROL_C: return XYZ( 10, 35, -22); // extra patrol during SURFACE_RELAY
      case SubState::HOLD_B:   return XYZ( 20, 40, -20);
    }
    return XYZ(0,0,-20);
  }
  geometry_msgs::msg::Point sub2Target(SubState s) const {
    switch(s){
      case SubState::HOLD_A:   return XYZ( 20, 60, -20);
      case SubState::FOLLOW_A: return XYZ( 35, 20, -20);
      case SubState::FOLLOW_B: return XYZ( 35, 60, -20);
      case SubState::PATROL_C: return XYZ( 10, 45, -22); // extra patrol during SURFACE_RELAY
      case SubState::HOLD_B:   return XYZ( 20, 40, -20);
    }
    return XYZ(0,0,-20);
  }

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

  // Called locally when we enter a new 0..4 slot (SURFACE_INIT/H1/H2/SURFACE_RELAY/RETURN)
  void onSlotEnter(const Sched& sch){
    // Reset per-slot publish flags so we only publish once in that slot
    published_this_slot_ = false;

    if (sch.my_role == Role::MID_TIER) {
      switch (sch.mid_state) {
        case MidState::SURFACE_INIT:
          // Start-of-cycle surface: seed plans, reset per-cycle added counters, and LOG it.
          RCLCPP_INFO(get_logger(),
            "[%s] MID SURFACE_INIT: surfacing — uploading known plans (s1=%zu, s2=%zu, mid=%zu); downloading latest",
            sub_id_.c_str(), known_sub1_.size(), known_sub2_.size(), known_mid_.size());
          ensureSeedPlans();
          resetAddedSinceSurface();
          break;

        case MidState::HANDOFF_1:
          // Talk to current SUB1 (actual node id in log)
          midHandoffOnce("sub1", sch.id_s1);
          break;

        case MidState::HANDOFF_2:
          // Talk to current SUB2 (actual node id in log)
          midHandoffOnce("sub2", sch.id_s2);
          break;

        case MidState::SURFACE_RELAY:
          // Second surface: relay (publish) the merged knowledge and log the new info gathered
          RCLCPP_INFO(get_logger(),
            "[%s] MID SURFACE_RELAY: relaying merged knowledge (new since last surface: s1=%zu, s2=%zu, mid=%zu)",
            sub_id_.c_str(), added_since_surface_sub1_, added_since_surface_sub2_, added_since_surface_mid_);
          surfaceRelayAggregate();
          // After relaying, reset counters to accumulate for next cycle
          resetAddedSinceSurface();
          break;

        case MidState::RETURN_END:
          RCLCPP_INFO(get_logger(), "[%s] MID RETURN_END: navigating to end-of-line / prep role rotation", sub_id_.c_str());
          break;
      }
    } else {
      // (optional) per-sub logs could go here
    }
  }

  //  MID: one-shot handoff publisher in the slot (logs actual recipient id)
  void midHandoffOnce(const std::string& role_key, const std::string& recipient_id){
    if (published_this_slot_) return;
    published_this_slot_ = true;

    RCLCPP_INFO(get_logger(),
      "[%s] MID handoff → %s (role=%s): publishing /plans/%s and /plans/mid; echoing /echo/mid",
      sub_id_.c_str(), recipient_id.c_str(), role_key.c_str(), role_key.c_str());

    // Publish to the intended recipient only + always publish mid plan
    if (role_key == "sub1") {
      plans_pub_sub1_->publish(toPoseArray(known_sub1_, "mid"));
    } else {
      plans_pub_sub2_->publish(toPoseArray(known_sub2_, "mid"));
    }
    plans_pub_mid_->publish (toPoseArray(known_mid_,  "mid"));

    // Echo MID's previous plan on canonical /echo/mid so others can merge MID's older knowledge
    if (!known_mid_prev_.empty()) {
      auto pa = toPoseArray(known_mid_prev_, "mid_prev");
      pa.header.stamp = this->get_clock()->now();
      echo_pub_mid_->publish(pa);
    }
    known_mid_prev_ = known_mid_;
  }

  // Any node: plan topic handler (for sub1/sub2/mid) 
  void onPlanTopic(const geometry_msgs::msg::PoseArray& pa, const std::string& topic_key /* "sub1"|"sub2"|"mid" */) {
    const std::string my_key = roleKeyNow();   // who am I right now?
    Plan new_plan = fromPoseArray(pa);

    std::size_t added = 0;
    if (topic_key=="sub1")      added = mergeIntoCount(known_sub1_, new_plan), added_since_surface_sub1_ += added;
    else if (topic_key=="sub2") added = mergeIntoCount(known_sub2_, new_plan), added_since_surface_sub2_ += added;
    else                        added = mergeIntoCount(known_mid_,  new_plan), added_since_surface_mid_  += added;

    if (topic_key == my_key) {// pick which one to publish 
      if (!my_prev_plan_.empty()) {
        auto echo_pa = toPoseArray(my_prev_plan_, my_key + "_prev");
        echo_pa.header.stamp = this->get_clock()->now();
        echoPubForRoleKey(my_key)->publish(echo_pa); // publish to /echo/sub1 or /echo/sub2 or /echo/mid
        RCLCPP_INFO(get_logger(), "[%s] ECHO %s prev plan (%zu pts)", sub_id_.c_str(), my_key.c_str(), my_prev_plan_.size());
      }
      my_prev_plan_ = my_plan_;
      my_plan_ = std::move(new_plan);
      RCLCPP_INFO(get_logger(), "[%s] MY PLAN updated for %s (%zu pts, +%zu new merged)", sub_id_.c_str(), my_key.c_str(), my_plan_.size(), added);
    }
  }

  //  Any node: on peer echo, merge into knowledge & count
  void onPeerEcho(const geometry_msgs::msg::PoseArray& pa, const std::string& who){
    Plan p = fromPoseArray(pa);
    std::size_t added = 0;
    if (who=="sub1")      added = mergeIntoCount(known_sub1_, p), added_since_surface_sub1_ += added;
    else if (who=="sub2") added = mergeIntoCount(known_sub2_, p), added_since_surface_sub2_ += added;
    else if (who=="mid")  added = mergeIntoCount(known_mid_,  p), added_since_surface_mid_  += added;
    RCLCPP_INFO(get_logger(), "[%s] Merged peer echo from %s (+%zu new pts)", sub_id_.c_str(), who.c_str(), added);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr& echoPubForRoleKey(const std::string& k){
    if (k=="sub1") return echo_pub_sub1_;
    if (k=="sub2") return echo_pub_sub2_;
    return echo_pub_mid_;
  }

  // SURFACE_RELAY: publish merged knowledge and log counts 
  void surfaceRelayAggregate(){
    // publish full merged knowledge (you can switch to deltas if desired)
    relay_pub_sub1_->publish(toPoseArray(known_sub1_, "relay"));
    relay_pub_sub2_->publish(toPoseArray(known_sub2_, "relay"));
    relay_pub_mid_->publish (toPoseArray(known_mid_,  "relay"));

    RCLCPP_INFO(get_logger(),
      "[%s] SURFACE_RELAY | relayed merged plans | new since last surface: sub1=%zu sub2=%zu mid=%zu",
      sub_id_.c_str(), added_since_surface_sub1_, added_since_surface_sub2_, added_since_surface_mid_);
  }

  void resetAddedSinceSurface(){
    added_since_surface_sub1_ = 0;
    added_since_surface_sub2_ = 0;
    added_since_surface_mid_  = 0;
  }

  //  Expected poses publisher (by role) 
  void publishExpectedPoses(double t){
    geometry_msgs::msg::PoseStamped ps; ps.header.stamp = this->get_clock()->now();
    geometry_msgs::msg::Point pt;
    if (expectedPoseAt(known_sub1_, t, pt)) { ps.header.frame_id="sub1"; ps.pose.position=pt; exp_pub_sub1_->publish(ps); }
    if (expectedPoseAt(known_sub2_, t, pt)) { ps.header.frame_id="sub2"; ps.pose.position=pt; exp_pub_sub2_->publish(ps); }
    if (expectedPoseAt(known_mid_,  t, pt)) { ps.header.frame_id="mid";  ps.pose.position=pt; exp_pub_mid_->publish(ps);  }
  }

  //  Seed some plans so handoffs have content (replace with file/param/IoT)
  void ensureSeedPlans(){
    if (known_mid_.empty()){
      // two rendezvous cycles, times aligned to dwell slots
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
  }

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

  Plan my_plan_;
  Plan my_prev_plan_;

  // Knowledge caches (what I think the world’s plans are)
  Plan known_sub1_;
  Plan known_sub2_;
  Plan known_mid_;
  Plan known_mid_prev_;

  // New points accumulated since last surface (for SURFACE_RELAY reporting)
  std::size_t added_since_surface_sub1_{0};
  std::size_t added_since_surface_sub2_{0};
  std::size_t added_since_surface_mid_{0};

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr plans_pub_sub1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr plans_pub_sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr plans_pub_mid_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr echo_pub_sub1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr echo_pub_sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr echo_pub_mid_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr relay_pub_sub1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr relay_pub_sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr relay_pub_mid_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exp_pub_sub1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exp_pub_sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exp_pub_mid_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr plans_sub_s1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr plans_sub_s2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr plans_sub_mid_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr echo_sub_s1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr echo_sub_s2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr echo_sub_mid_;

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
