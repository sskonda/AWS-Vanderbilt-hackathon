// #include <chrono>
// #include <string>
// #include <vector>
// #include <array>
// #include <memory>
// #include <algorithm>
// #include <unordered_map>
// #include <sstream>
// #include <cmath>
// #include <functional>
// #include <cstdint>

// #include <rclcpp/rclcpp.hpp>
// #include <builtin_interfaces/msg/time.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <nav_msgs/msg/path.hpp>
// #include <std_msgs/msg/string.hpp>

// // Custom messages (note: lowercase headers on Linux)
// #include "custom_msg/msg/plan.hpp"
// #include "custom_msg/msg/event.hpp"

// using namespace std::chrono_literals;

// // roles and states
// enum class Role     { SUB1, SUB2, MID_TIER };
// enum class MidState { SURFACE_INIT, HANDOFF_1, HANDOFF_2, SURFACE_RELAY, RETURN_END };
// enum class SubState { HOLD_A,  FOLLOW_A,     FOLLOW_B,   PATROL_C,      HOLD_B    };

// // helpers
// static geometry_msgs::msg::Point XYZ(double x,double y,double z){ geometry_msgs::msg::Point p; p.x=x;p.y=y;p.z=z; return p; }
// static double now_unix() {
//   return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
// }
// static builtin_interfaces::msg::Time toStamp(double t){
//   builtin_interfaces::msg::Time st;
//   if (t < 0) { st.sec = 0; st.nanosec = 0; return st; }
//   const auto sec = static_cast<int32_t>(std::floor(t));
//   const double frac = t - static_cast<double>(sec);
//   st.sec = sec;
//   st.nanosec = static_cast<uint32_t>(std::round(frac * 1e9));
//   return st;
// }
// static double fromStamp(const builtin_interfaces::msg::Time& st){
//   return static_cast<double>(st.sec) + static_cast<double>(st.nanosec) * 1e-9;
// }

// // plans + structures
// struct Waypoint { double t; double x,y,z; };   // t = unix seconds
// using PlanVec = std::vector<Waypoint>;

// // Convert between internal PlanVec and custom_msg/Plan
// static custom_msg::msg::Plan toPlanMsg(const PlanVec& plan, const std::string& frame)
// {
//   custom_msg::msg::Plan out;
//   out.header.frame_id = frame;
//   out.header.stamp = toStamp(now_unix());

//   nav_msgs::msg::Path path;
//   path.header.frame_id = frame;
//   path.header.stamp = out.header.stamp;

//   path.poses.reserve(plan.size());
//   for(const auto& w: plan){
//     geometry_msgs::msg::PoseStamped ps;
//     ps.header.frame_id = frame;
//     ps.header.stamp = toStamp(w.t);
//     ps.pose.position.x = w.x;
//     ps.pose.position.y = w.y;
//     ps.pose.position.z = w.z;
//     ps.pose.orientation.w = 1.0; // identity orientation; not used in planning
//     path.poses.push_back(std::move(ps));
//   }
//   out.paths.push_back(std::move(path));
//   return out;
// }

// static PlanVec fromPlanMsg(const custom_msg::msg::Plan& msg)
// {
//   PlanVec plan;
//   for(const auto& path : msg.paths){
//     for(const auto& ps : path.poses){
//       plan.push_back(Waypoint{
//         fromStamp(ps.header.stamp),
//         ps.pose.position.x,
//         ps.pose.position.y,
//         ps.pose.position.z
//       });
//     }
//   }
//   std::sort(plan.begin(), plan.end(), [](auto&a,auto&b){return a.t<b.t;});
//   return plan;
// }

// // merge add into base; return number of newly added points (exact match on t,x,y,z)
// static std::size_t mergeIntoCount(PlanVec& base, const PlanVec& add){
//   std::size_t newc = 0;
//   for(const auto& w: add){
//     bool has=false;
//     for(const auto& v: base){
//       if (std::fabs(v.t-w.t)<1e-9 && std::fabs(v.x-w.x)<1e-9 && std::fabs(v.y-w.y)<1e-9 && std::fabs(v.z-w.z)<1e-9){ has=true; break; }
//     }
//     if(!has){ base.push_back(w); ++newc; }
//   }
//   std::sort(base.begin(), base.end(), [](auto&a,auto&b){return a.t<b.t;});
//   return newc;
// }

// // sanitize: strictly-increasing timestamps (no duplicates) and drop huge jumps
// static void sanitizePlanHistory(PlanVec& v, double max_jump_m = 30.0){
//   if (v.empty()) return;
//   std::sort(v.begin(), v.end(), [](const Waypoint& a, const Waypoint& b){ return a.t < b.t; });
//   PlanVec out; out.reserve(v.size());
//   const double eps = 1e-6; // 1 microsecond
//   double last_t = -1e300;
//   bool have_prev = false;
//   Waypoint prev{};
//   for (auto w : v){
//     if (w.t <= last_t) w.t = last_t + eps; // make strictly increasing
//     bool ok = true;
//     if (have_prev){
//       const double dx = w.x - prev.x;
//       const double dy = w.y - prev.y;
//       const double dz = w.z - prev.z;
//       const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
//       if (dist > max_jump_m) ok = false; // drop “teleport”
//     }
//     if (ok){
//       out.push_back(w);
//       prev = w;
//       last_t = w.t;
//       have_prev = true;
//     }
//   }
//   v.swap(out);
// }

// // linear interpolation of expected pose at time t; fallback to nearest
// static bool expectedPoseAt(const PlanVec& plan, double t, geometry_msgs::msg::Point& out){
//   if (plan.empty()) return false;
//   if (t <= plan.front().t){ out = XYZ(plan.front().x, plan.front().y, plan.front().z); return true; }
//   if (t >= plan.back().t) { out = XYZ(plan.back().x,  plan.back().y,  plan.back().z);  return true; }
//   for (std::size_t i=1;i<plan.size();++i){
//     if (t <= plan[i].t){
//       const auto& a = plan[i-1]; const auto& b = plan[i];
//       const double denom = std::max(1e-9, (b.t - a.t));
//       const double u = (t - a.t) / denom;
//       out = XYZ(a.x + u*(b.x-a.x), a.y + u*(b.y-a.y), a.z + u*(b.z-a.z));
//       return true;
//     }
//   }
//   return false;
// }

// class SubNode : public rclcpp::Node {
// public:
//   explicit SubNode(const std::string& sub_id)
//   : Node(sub_id + "_node"), sub_id_(sub_id)
//   {
//     // parameters
//     this->declare_parameter<std::vector<std::string>>("team_ids",   {"subMID","subA","subB"});
//     this->declare_parameter<std::string>("leader_id", "subMID");
//     this->declare_parameter<double>("start_epoch",    0.0);
//     this->declare_parameter<double>("state_dwell_s",  20.0);
//     this->declare_parameter<bool>  ("publish_current_pose", false);
//     this->declare_parameter<int64_t>("Identifier", -1);
//     // Optional: identity numbers aligned with team_ids (stable across role rotation)
//     this->declare_parameter<std::vector<int64_t>>("team_identity_numbers", {0,1,2});

//     Identifier = this->get_parameter("Identifier").as_int();  // store my stable Identifier number
//     team_ids_            = this->get_parameter("team_ids").as_string_array();
//     leader_id_           = this->get_parameter("leader_id").as_string();
//     start_epoch_         = this->get_parameter("start_epoch").as_double();
//     dwell_s_             = this->get_parameter("state_dwell_s").as_double();
//     publish_current_pose_= this->get_parameter("publish_current_pose").as_bool();
//     team_identity_numbers_ = this->get_parameter("team_identity_numbers").as_integer_array();

//     // Ensure presence + dedupe of my sub_id in team_ids
//     if (std::find(team_ids_.begin(), team_ids_.end(), sub_id_) == team_ids_.end())
//       team_ids_.push_back(sub_id_);
//     { // dedupe preserving first occurrence
//       std::vector<std::string> uniq; uniq.reserve(team_ids_.size());
//       for (auto& s: team_ids_) if (std::find(uniq.begin(), uniq.end(), s)==uniq.end()) uniq.push_back(s);
//       team_ids_.swap(uniq);
//     }

//     // Align identity_numbers vector length to team_ids
//     if (team_identity_numbers_.size() != team_ids_.size()){
//       // fallback: identity = index
//       team_identity_numbers_.resize(team_ids_.size());
//       for (size_t i=0;i<team_ids_.size();++i) team_identity_numbers_[i] = static_cast<int64_t>(i);
//     }

//     // Build mapping team_id -> identity number
//     for (size_t i=0;i<team_ids_.size(); ++i){
//       idnum_by_teamid_[team_ids_[i]] = team_identity_numbers_[i];
//     }

//     auto it = std::find(team_ids_.begin(), team_ids_.end(), leader_id_);
//     if (it == team_ids_.end()) {
//       RCLCPP_WARN(get_logger(), "leader_id '%s' not in team_ids; using team_ids[0]", leader_id_.c_str());
//       leader0_idx_ = 0; leader_id_ = team_ids_.front();
//     } else {
//       leader0_idx_ = static_cast<int>(std::distance(team_ids_.begin(), it));
//     }

//     if (start_epoch_ <= 0.0) {
//       start_epoch_ = std::ceil(now_unix()) + 2.0;
//       RCLCPP_INFO(get_logger(), "[%s] start_epoch not set; using %.0f", sub_id_.c_str(), start_epoch_);
//     }

//     // Publishers
//     target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/" + sub_id_ + "/target_pose", 10);
//     if (publish_current_pose_) pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/" + sub_id_ + "/pose", 10);

//     // Plan pubs (custom Plan) by ROLE topics (existing contract)
//     plans_pub_sub1_ = create_publisher<custom_msg::msg::Plan>("/plans/sub1", 10);
//     plans_pub_sub2_ = create_publisher<custom_msg::msg::Plan>("/plans/sub2", 10);
//     plans_pub_mid_  = create_publisher<custom_msg::msg::Plan>("/plans/mid",  10);

//     echo_pub_sub1_ = create_publisher<custom_msg::msg::Plan>("/echo/sub1", 10);
//     echo_pub_sub2_ = create_publisher<custom_msg::msg::Plan>("/echo/sub2", 10);
//     echo_pub_mid_  = create_publisher<custom_msg::msg::Plan>("/echo/mid",  10);

//     // Relay pubs
//     relay_pub_sub1_ = create_publisher<custom_msg::msg::Plan>("/relay/sub1", 10);
//     relay_pub_sub2_ = create_publisher<custom_msg::msg::Plan>("/relay/sub2", 10);
//     relay_pub_mid_  = create_publisher<custom_msg::msg::Plan>("/relay/mid",  10);
//     relay_snapshot_json_pub_ = create_publisher<std_msgs::msg::String>("/relay/snapshot_json", 10);
//     relay_events_json_pub_   = create_publisher<std_msgs::msg::String>("/relay/events_json",   10);

//     // Expected pose pubs
//     exp_pub_sub1_ = create_publisher<geometry_msgs::msg::PoseStamped>("/expected/sub1/pose", 10);
//     exp_pub_sub2_ = create_publisher<geometry_msgs::msg::PoseStamped>("/expected/sub2/pose", 10);
//     exp_pub_mid_  = create_publisher<geometry_msgs::msg::PoseStamped>("/expected/mid/pose",  10);

//     // Event pubs (per-role topics)
//     event_pub_sub1_ = create_publisher<custom_msg::msg::Event>("/events/sub1", 10);
//     event_pub_sub2_ = create_publisher<custom_msg::msg::Event>("/events/sub2", 10);
//     event_pub_mid_  = create_publisher<custom_msg::msg::Event>("/events/mid",  10);

//     // subscriptions: Plans (custom Plan) by ROLE topics
//     plans_sub_s1_ = create_subscription<custom_msg::msg::Plan>(
//       "/plans/sub1", 10,
//       [this](custom_msg::msg::Plan::ConstSharedPtr msg){
//         this->onPlanTopic(*msg, "sub1");
//       });

//     plans_sub_s2_ = create_subscription<custom_msg::msg::Plan>(
//       "/plans/sub2", 10,
//       [this](custom_msg::msg::Plan::ConstSharedPtr msg){
//         this->onPlanTopic(*msg, "sub2");
//       });

//     plans_sub_mid_ = create_subscription<custom_msg::msg::Plan>(
//       "/plans/mid", 10,
//       [this](custom_msg::msg::Plan::ConstSharedPtr msg){
//         this->onPlanTopic(*msg, "mid");
//       });

//     // Echo (Plan) listeners
//     echo_sub_s1_ = create_subscription<custom_msg::msg::Plan>(
//       "/echo/sub1", 10,
//       [this](custom_msg::msg::Plan::ConstSharedPtr msg){
//         this->onPeerEcho(*msg, "sub1");
//       });

//     echo_sub_s2_ = create_subscription<custom_msg::msg::Plan>(
//       "/echo/sub2", 10,
//       [this](custom_msg::msg::Plan::ConstSharedPtr msg){
//         this->onPeerEcho(*msg, "sub2");
//       });

//     echo_sub_mid_ = create_subscription<custom_msg::msg::Plan>(
//       "/echo/mid", 10,
//       [this](custom_msg::msg::Plan::ConstSharedPtr msg){
//         this->onPeerEcho(*msg, "mid");
//       });

//     // Events for each role
//     event_sub_s1_ = create_subscription<custom_msg::msg::Event>(
//       "/events/sub1", 10,
//       [this](custom_msg::msg::Event::ConstSharedPtr msg){
//         this->onEvent(*msg, "sub1");
//       });

//     event_sub_s2_ = create_subscription<custom_msg::msg::Event>(
//       "/events/sub2", 10,
//       [this](custom_msg::msg::Event::ConstSharedPtr msg){
//         this->onEvent(*msg, "sub2");
//       });

//     event_sub_mid_ = create_subscription<custom_msg::msg::Event>(
//       "/events/mid", 10,
//       [this](custom_msg::msg::Event::ConstSharedPtr msg){
//         this->onEvent(*msg, "mid");
//       });

//     timer_ = create_wall_timer(std::chrono::duration<double>(0.25), std::bind(&SubNode::tick, this));

//     RCLCPP_INFO(get_logger(),
//       "[%s] ring=%zu leader0=%s(idx=%d) epoch=%.0f dwell=%.1fs",
//       sub_id_.c_str(), team_ids_.size(), leader_id_.c_str(), leader0_idx_, start_epoch_, dwell_s_);
//     RCLCPP_INFO(get_logger(),
//       "[%s] id=%ld ring=%zu leader0=%s(idx=%d) epoch=%.0f dwell=%.1fs",
//       sub_id_.c_str(), static_cast<long>(Identifier),
//       team_ids_.size(), leader_id_.c_str(), leader0_idx_, start_epoch_, dwell_s_);
//   }

// private:
//   int64_t Identifier{-1};
//   std::vector<int64_t> team_identity_numbers_;                 // aligned to team_ids_
//   std::unordered_map<std::string,int64_t> idnum_by_teamid_;    // team_id -> identity number

//   struct Sched {
//     Role my_role;
//     MidState mid_state;    // valid if my_role==MID_TIER
//     SubState sub_state;    // valid if my_role!=MID_TIER
//     std::string id_mid, id_s1, id_s2; // team_ids of role holders at time t
//     int slot5;
//   };

//   Sched compute(double tunix) const {
//     const int N = static_cast<int>(team_ids_.size());
//     const double T = 5.0 * dwell_s_;
//     const double dt = std::max(0.0, tunix - start_epoch_);
//     const long rot_k = (T>0.0)? static_cast<long>(std::floor(dt / T)) : 0L;

//     const int leader_idx = (leader0_idx_ + static_cast<int>(rot_k % N) + N) % N;
//     const int s1_idx = (leader_idx + 1) % N;
//     const int s2_idx = (leader_idx + 2) % N;

//     const std::string& mid = team_ids_[leader_idx];
//     const std::string& s1  = team_ids_[s1_idx];
//     const std::string& s2  = team_ids_[s2_idx];

//     const long slot = (dwell_s_>0.0)? static_cast<long>(std::floor(std::fmod(dt, T) / dwell_s_)) : 0L;
//     const int slot5 = static_cast<int>((slot % 5 + 5) % 5);

//     MidState ms = MidState::SURFACE_INIT;
//     if      (slot5==1) ms=MidState::HANDOFF_1;
//     else if (slot5==2) ms=MidState::HANDOFF_2;
//     else if (slot5==3) ms=MidState::SURFACE_RELAY;
//     else if (slot5==4) ms=MidState::RETURN_END;

//     SubState ss = SubState::HOLD_A;
//     if      (slot5==1) ss=SubState::FOLLOW_A;
//     else if (slot5==2) ss=SubState::FOLLOW_B;
//     else if (slot5==3) ss=SubState::PATROL_C;
//     else if (slot5==4) ss=SubState::HOLD_B;

//     Role r=Role::SUB1;
//     if (sub_id_==mid) r=Role::MID_TIER;
//     else if (sub_id_==s1) r=Role::SUB1;
//     else if (sub_id_==s2) r=Role::SUB2;

//     return Sched{r,ms,ss,mid,s1,s2,slot5};
//   }

//   // lookup helpers
//   std::string roleKeyNow() const {
//     const auto sch = compute(now_unix());
//     if (sch.my_role == Role::MID_TIER) return "mid";
//     if (sch.my_role == Role::SUB1)     return "sub1";
//     return "sub2";
//   }
//   static std::string roleOfTeamId(const std::string& team_id, const Sched& sch){
//     if (team_id == sch.id_mid) return "mid";
//     if (team_id == sch.id_s1)  return "sub1";
//     if (team_id == sch.id_s2)  return "sub2";
//     return "none";
//   }
//   int64_t identityOfTeamId(const std::string& team_id) const {
//     auto it = idnum_by_teamid_.find(team_id);
//     if (it != idnum_by_teamid_.end()) return it->second;
//     return -1;
//   }

//   geometry_msgs::msg::Point midTarget(MidState s) const {
//     switch(s){
//       case MidState::SURFACE_INIT: return XYZ(  0,  0,  -2);
//       case MidState::HANDOFF_1:    return XYZ( 50, 20, -10);
//       case MidState::HANDOFF_2:    return XYZ( 50, 60, -10);
//       case MidState::SURFACE_RELAY:return XYZ(  0,  5,  -2);
//       case MidState::RETURN_END:   return XYZ(100, 40, -10);
//     }
//     return XYZ(0,0,-5);
//   }
//   geometry_msgs::msg::Point sub1Target(SubState s) const {
//     switch(s){
//       case SubState::HOLD_A:   return XYZ( 20, 20, -20);
//       case SubState::FOLLOW_A: return XYZ( 35, 20, -20);
//       case SubState::FOLLOW_B: return XYZ( 35, 60, -20);
//       case SubState::PATROL_C: return XYZ( 10, 35, -22);
//       case SubState::HOLD_B:   return XYZ( 20, 40, -20);
//     }
//     return XYZ(0,0,-20);
//   }
//   geometry_msgs::msg::Point sub2Target(SubState s) const {
//     switch(s){
//       case SubState::HOLD_A:   return XYZ( 20, 60, -20);
//       case SubState::FOLLOW_A: return XYZ( 35, 20, -20);
//       case SubState::FOLLOW_B: return XYZ( 35, 60, -20);
//       case SubState::PATROL_C: return XYZ( 10, 45, -22);
//       case SubState::HOLD_B:   return XYZ( 20, 40, -20);
//     }
//     return XYZ(0,0,-20);
//   }

//   // publish my event at SURFACE_INIT (non-MID roles)
//   void publishMyEventIfNeeded(const Sched& sch)
//   {
//     if (sch.my_role == Role::MID_TIER) return;

//     custom_msg::msg::Event ev;
//     ev.header.stamp = toStamp(now_unix());
//     ev.header.frame_id = sub_id_;
//     ev.waypoints.clear();

//     geometry_msgs::msg::PoseStamped ps;
//     ps.header = ev.header;
//     ps.pose.position = (sch.my_role==Role::SUB1) ? sub1Target(sch.sub_state) : sub2Target(sch.sub_state);
//     ps.pose.orientation.w = 1.0;
//     ev.waypoints.push_back(ps);

//     ev.event_type = last_local_event_type_.empty() ? "nothing_new" : last_local_event_type_;

//     if (sch.my_role==Role::SUB1) event_pub_sub1_->publish(ev);
//     else                         event_pub_sub2_->publish(ev);

//     RCLCPP_INFO(get_logger(), "[%s] Published SURFACE_INIT event on /events/%s: %s",
//       sub_id_.c_str(),
//       (sch.my_role==Role::SUB1?"sub1":"sub2"),
//       ev.event_type.c_str());
//   }

//   void onEvent(const custom_msg::msg::Event& ev, const std::string& role_key){
//     latest_events_[role_key] = ev;
//     ++events_since_surface_[role_key];
//     RCLCPP_INFO(get_logger(), "[%s] Received Event from %s: %s", sub_id_.c_str(), role_key.c_str(), ev.event_type.c_str());
//   }

//   // make and publish events JSON uplink at SURFACE_INIT if it is the mid node
//   void uplinkEventsJSON(const Sched& sch){
//     const double jstime = now_unix();
//     std::ostringstream js;
//     js << "{"
//       << "\"kind\":\"surface_relay_snapshot\","
//       << "\"sender\":\"" << sub_id_ << "_node\","
//       << "\"Identifier\":" << Identifier << ","
//       << "\"role\":\"MID_TIER\","
//       << "\"cycle\":{"
//       <<   "\"epoch\":" << std::fixed << start_epoch_ << ","
//       <<   "\"dwell_s\":" << dwell_s_ << ","
//       <<   "\"slot\":" << sch.slot5 << "},"
//       << "\"events\":{";

//     auto writeEv = [&](const std::string& key){
//       js << "\"" << key << "\":{";
//       if (latest_events_.count(key)){
//         const auto& ev = latest_events_[key];
//         double x=0,y=0,z=0;
//         if (!ev.waypoints.empty()){
//           const auto& p = ev.waypoints.front().pose.position;
//           x=p.x; y=p.y; z=p.z;
//         }
//         js << "\"event_type\":\"" << ev.event_type << "\","
//            << "\"x\":" << x << ",\"y\":" << y << ",\"z\":" << z << "}";
//       } else {
//         js << "\"event_type\":\"nothing_new\",\"x\":0,\"y\":0,\"z\":0}";
//       }
//     };

//     writeEv("mid");   js << ",";
//     writeEv("sub1");  js << ",";
//     writeEv("sub2");

//     js << "},"
//        << "\"new_counts\":{"
//        <<   "\"mid\":"  << events_since_surface_["mid"]  << ","
//        <<   "\"sub1\":" << events_since_surface_["sub1"] << ","
//        <<   "\"sub2\":" << events_since_surface_["sub2"] << "},"
//        << "\"sent_at\":" << std::fixed << jstime
//        << "}";

//     std_msgs::msg::String out;
//     out.data = js.str();
//     relay_events_json_pub_->publish(out);

//     events_since_surface_["mid"]  = 0;
//     events_since_surface_["sub1"] = 0;
//     events_since_surface_["sub2"] = 0;

//     RCLCPP_INFO(get_logger(), "[%s] Uplinked events JSON at SURFACE_INIT", sub_id_.c_str());
//   }

//   // plans per tick, state cycles
//   void tick(){
//     const double t = now_unix();
//     const auto sch = compute(t);

//     if (sch.slot5 != last_slot5_ || sch.id_mid != last_mid_id_) {
//       onSlotEnter(sch);
//       last_slot5_ = sch.slot5;
//       last_mid_id_ = sch.id_mid;
//     }

//     // Publish nav target per role/state
//     geometry_msgs::msg::PoseStamped tgt;
//     tgt.header.stamp = this->get_clock()->now();
//     tgt.header.frame_id = sub_id_;
//     if (sch.my_role == Role::MID_TIER) {
//       tgt.pose.position = midTarget(sch.mid_state);
//     } else if (sch.my_role == Role::SUB1) {
//       tgt.pose.position = sub1Target(sch.sub_state);
//     } else {
//       tgt.pose.position = sub2Target(sch.sub_state);
//     }
//     target_pub_->publish(tgt);
//     if (publish_current_pose_) { if (pose_pub_) pose_pub_->publish(tgt); }

//     publishExpectedPoses(t);

//     if (static_cast<int>(t) % 2 == 0 && (last_log_second_ != static_cast<int>(t))) {
//       last_log_second_ = static_cast<int>(t);
//       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
//         "[%s] role=%s mid=%s s1=%s s2=%s slot=%d dwell=%.1f (added: s1=%zu s2=%zu mid=%zu)",
//         sub_id_.c_str(),
//         (sch.my_role==Role::MID_TIER?"MID":(sch.my_role==Role::SUB1?"SUB1":"SUB2")),
//         sch.id_mid.c_str(), sch.id_s1.c_str(), sch.id_s2.c_str(),
//         sch.slot5, dwell_s_,
//         added_since_surface_sub1_, added_since_surface_sub2_, added_since_surface_mid_);
//     }
//   }

//   // Called locally when we enter a new 0..4 slot
//   void onSlotEnter(const Sched& sch){
//     published_this_slot_ = false;

//     // At the start of each SURFACE_INIT, non-MID subs publish their current event
//     if (sch.mid_state == MidState::SURFACE_INIT) {
//       publishMyEventIfNeeded(sch);
//     }

//     if (sch.my_role == Role::MID_TIER) {
//       switch (sch.mid_state) {
//         case MidState::SURFACE_INIT:
//           RCLCPP_INFO(get_logger(),
//             "[%s] MID SURFACE_INIT: surfacing — uploading known plans (s1=%zu, s2=%zu, mid=%zu); requesting/latest events",
//             sub_id_.c_str(), known_sub1_.size(), known_sub2_.size(), known_mid_.size());
//           ensureSeedPlans();
//           resetAddedSinceSurface();
//           uplinkEventsJSON(sch);
//           break;

//         case MidState::HANDOFF_1:
//           midHandoffOnce("sub1", sch.id_s1);
//           break;

//         case MidState::HANDOFF_2:
//           midHandoffOnce("sub2", sch.id_s2);
//           break;

//         case MidState::SURFACE_RELAY:
//           RCLCPP_INFO(get_logger(),
//             "[%s] MID SURFACE_RELAY: relaying merged knowledge (new since last surface: s1=%zu, s2=%zu, mid=%zu)",
//             sub_id_.c_str(), added_since_surface_sub1_, added_since_surface_sub2_, added_since_surface_mid_);
//           surfaceRelayAggregate();
//           resetAddedSinceSurface();
//           break;

//         case MidState::RETURN_END:
//           RCLCPP_INFO(get_logger(), "[%s] MID RETURN_END: navigating to end-of-line / prep role rotation", sub_id_.c_str());
//           break;
//       }
//     }
//   }

//   // MID: one-shot handoff publisher in the slot
//   void midHandoffOnce(const std::string& role_key, const std::string& recipient_id){
//     if (published_this_slot_) return;
//     published_this_slot_ = true;

//     RCLCPP_INFO(get_logger(),
//       "[%s] MID handoff → %s (role=%s): publishing /plans/%s and /plans/mid; echoing /echo/mid",
//       sub_id_.c_str(), recipient_id.c_str(), role_key.c_str(), role_key.c_str());

//     if (role_key == "sub1") {
//       plans_pub_sub1_->publish(toPlanMsg(known_sub1_, "mid"));
//     } else {
//       plans_pub_sub2_->publish(toPlanMsg(known_sub2_, "mid"));
//     }
//     plans_pub_mid_->publish (toPlanMsg(known_mid_,  "mid"));

//     if (!known_mid_prev_.empty()) {
//       auto pa = toPlanMsg(known_mid_prev_, "mid_prev");
//       echo_pub_mid_->publish(pa);
//     }
//     known_mid_prev_ = known_mid_;
//   }

//   // ROLE-topic plan reception → merge into role knowledge AND per-identity store
//   void onPlanTopic(const custom_msg::msg::Plan& pm, const std::string& topic_key /* "sub1"|"sub2"|"mid" */) {
//     const std::string my_key = roleKeyNow();
//     PlanVec new_plan = fromPlanMsg(pm);

//     // Merge into role-based knowledge (kept for expected poses + legacy pubs)
//     std::size_t added = 0;
//     if (topic_key=="sub1")      added = mergeIntoCount(known_sub1_, new_plan), added_since_surface_sub1_ += added;
//     else if (topic_key=="sub2") added = mergeIntoCount(known_sub2_, new_plan), added_since_surface_sub2_ += added;
//     else                        added = mergeIntoCount(known_mid_,  new_plan), added_since_surface_mid_  += added;

//     // Also merge into the PER-IDENTITY store for *whoever currently holds that role*
//     const auto sch = compute(now_unix());
//     std::string team_id_holder = (topic_key=="mid")? sch.id_mid : (topic_key=="sub1"? sch.id_s1 : sch.id_s2);
//     int64_t idnum = identityOfTeamId(team_id_holder);
//     if (idnum >= 0){
//       mergeIntoCount(per_identity_plans_[idnum], new_plan);
//       // keep the per-identity history == merged plans (we sanitize when serializing)
//     }

//     // If this plan is for me-in-my-current-role, do echo bookkeeping
//     if (topic_key == my_key) {
//       if (!my_prev_plan_.empty()) {
//         auto echo_pa = toPlanMsg(my_prev_plan_, my_key + std::string("_prev"));
//         echo_pub_for_key(my_key)->publish(echo_pa);
//         RCLCPP_INFO(get_logger(), "[%s] ECHO %s prev plan (%zu pts)", sub_id_.c_str(), my_key.c_str(), my_prev_plan_.size());
//       }
//       my_prev_plan_ = my_plan_;
//       my_plan_ = std::move(new_plan);
//       RCLCPP_INFO(get_logger(), "[%s] MY PLAN updated for %s (%zu pts, +%zu new merged)", sub_id_.c_str(), my_key.c_str(), my_plan_.size(), added);
//     }
//   }

//   // Any node: on peer echo, merge into knowledge & counters (role-based only)
//   void onPeerEcho(const custom_msg::msg::Plan& pm, const std::string& who){
//     PlanVec p = fromPlanMsg(pm);
//     std::size_t added = 0;
//     if (who=="sub1")      added = mergeIntoCount(known_sub1_, p), added_since_surface_sub1_ += added;
//     else if (who=="sub2") added = mergeIntoCount(known_sub2_, p), added_since_surface_sub2_ += added;
//     else if (who=="mid")  added = mergeIntoCount(known_mid_,  p), added_since_surface_mid_  += added;
//     RCLCPP_INFO(get_logger(), "[%s] Merged peer echo from %s (+%zu new pts)", sub_id_.c_str(), who.c_str(), added);
//   }

//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr& echo_pub_for_key(const std::string& k){
//     if (k=="sub1") return echo_pub_sub1_;
//     if (k=="sub2") return echo_pub_sub2_;
//     return echo_pub_mid_;
//   }

//   // JSON helper to serialize a plan/history array
//   static std::string planToJsonArray(const PlanVec& p){
//     std::ostringstream os; os << "[";
//     for (size_t i=0;i<p.size();++i){
//       const auto& w=p[i];
//       os << "{\"t\":"<<std::fixed<<w.t<<",\"x\":"<<w.x<<",\"y\":"<<w.y<<",\"z\":"<<w.z<<"}";
//       if(i+1<p.size()) os << ",";
//     }
//     os << "]";
//     return os.str();
//   }

//   // SURFACE_RELAY: publish merged knowledge + JSON snapshot keyed by Identity
//   void surfaceRelayAggregate(){
//     // publish role-based merged knowledge (unchanged)
//     relay_pub_sub1_->publish(toPlanMsg(known_sub1_, "relay"));
//     relay_pub_sub2_->publish(toPlanMsg(known_sub2_, "relay"));
//     relay_pub_mid_->publish (toPlanMsg(known_mid_,  "relay"));

//     const auto sch = compute(now_unix());

//     // Build per-identity sanitized copies (history == plans)
//     // Ensure we have entries for all team members
//     for (const auto& team_id : team_ids_){
//       int64_t idn = identityOfTeamId(team_id);
//       (void) per_identity_plans_[idn]; // ensure default-created
//     }

//     // Create sanitized copies and (optionally) cap extreme jumps
//     std::unordered_map<int64_t, PlanVec> plans_sanitized;
//     std::unordered_map<int64_t, PlanVec> history_sanitized;
//     for (const auto& team_id : team_ids_){
//       const int64_t idn = identityOfTeamId(team_id);
//       PlanVec cp = per_identity_plans_[idn];     // copy
//       sanitizePlanHistory(cp, 30.0);
//       plans_sanitized[idn]   = cp;
//       history_sanitized[idn] = cp;               // history == plans
//     }

//     // Also still provide the role-based arrays for backward compatibility
//     PlanVec k_mid = known_mid_, k_s1 = known_sub1_, k_s2 = known_sub2_;
//     sanitizePlanHistory(k_mid, 30.0);
//     sanitizePlanHistory(k_s1,  30.0);
//     sanitizePlanHistory(k_s2,  30.0);

//     // Build JSON
//     std::ostringstream js;
//     js << "{"
//        << "\"kind\":\"surface_relay_snapshot\","
//        << "\"sender\":\"" << sub_id_ << "_node\","
//        << "\"Identifier\":" << Identifier << ","
//        << "\"role\":\"MID_TIER\","
//        << "\"cycle\":{"
//        <<   "\"epoch\":" << std::fixed << start_epoch_ << ","
//        <<   "\"dwell_s\":" << dwell_s_ << ","
//        <<   "\"slot\":" << sch.slot5 << ","
//        <<   "\"ids\":{"
//        <<     "\"mid\":\""  << sch.id_mid << "\","
//        <<     "\"sub1\":\"" << sch.id_s1  << "\","
//        <<     "\"sub2\":\"" << sch.id_s2  << "\"}"
//        << "},"
//        << "\"stats\":{"
//        <<   "\"new_since_last_surface\":{"
//        <<     "\"sub1\":" << added_since_surface_sub1_ << ","
//        <<     "\"sub2\":" << added_since_surface_sub2_ << ","
//        <<     "\"mid\":"  << added_since_surface_mid_  << "}"
//        << "},"
//        << "\"plans_by_role\":{"
//        <<   "\"mid\":"  << planToJsonArray(k_mid) << ","
//        <<   "\"sub1\":" << planToJsonArray(k_s1)  << ","
//        <<   "\"sub2\":" << planToJsonArray(k_s2)  << "},"
//        << "\"by_identity\":{";

//     // by_identity: "IDNUM": { "team_id": "...", "role_now":"...", "plan":[...], "history":[...] }
//     for (size_t i=0;i<team_ids_.size(); ++i){
//       const std::string& team_id = team_ids_[i];
//       const int64_t idn = team_identity_numbers_[i];
//       const std::string role_now = roleOfTeamId(team_id, sch);
//       const PlanVec& p  = plans_sanitized[idn];
//       const PlanVec& h  = history_sanitized[idn];
//       js << "\"" << idn << "\":{"
//          <<   "\"team_id\":\"" << team_id << "\","
//          <<   "\"role_now\":\"" << role_now << "\","
//          <<   "\"plan\":"    << planToJsonArray(p) << ","
//          <<   "\"history\":" << planToJsonArray(h) << "}";
//       if (i + 1 < team_ids_.size()) js << ",";
//     }
//     js << "},"
//        << "\"sent_at\":" << std::fixed << now_unix()
//        << "}";

//     std_msgs::msg::String out;
//     out.data = js.str();
//     relay_snapshot_json_pub_->publish(out);

//     RCLCPP_INFO(get_logger(),
//       "[%s] SURFACE_RELAY | relayed merged plans (by role + by identity) | new since last surface: sub1=%zu sub2=%zu mid=%zu",
//       sub_id_.c_str(), added_since_surface_sub1_, added_since_surface_sub2_, added_since_surface_mid_);
//   }

//   void resetAddedSinceSurface(){
//     added_since_surface_sub1_ = 0;
//     added_since_surface_sub2_ = 0;
//     added_since_surface_mid_  = 0;
//   }

//   // Expected poses publisher (by role)
//   void publishExpectedPoses(double t){
//     geometry_msgs::msg::PoseStamped ps; ps.header.stamp = this->get_clock()->now();
//     geometry_msgs::msg::Point pt;
//     if (expectedPoseAt(known_sub1_, t, pt)) { ps.header.frame_id="sub1"; ps.pose.position=pt; exp_pub_sub1_->publish(ps); }
//     if (expectedPoseAt(known_sub2_, t, pt)) { ps.header.frame_id="sub2"; ps.pose.position=pt; exp_pub_sub2_->publish(ps); }
//     if (expectedPoseAt(known_mid_,  t, pt)) { ps.header.frame_id="mid";  ps.pose.position=pt; exp_pub_mid_->publish(ps);  }
//   }

//   // Seed some plans so handoffs have content (also seed per-identity from current role holders)
//   void ensureSeedPlans(){
//     if (known_mid_.empty()){
//       known_mid_ = {
//         {start_epoch_ +  1*dwell_s_, 50,20,-10},
//         {start_epoch_ +  2*dwell_s_, 50,60,-10},
//         {start_epoch_ +  6*dwell_s_, 50,20,-10},
//         {start_epoch_ +  7*dwell_s_, 50,60,-10},
//       };
//     }
//     if (known_sub1_.empty()){
//       known_sub1_ = {
//         {start_epoch_ +  1*dwell_s_, 35,20,-20},
//         {start_epoch_ +  2*dwell_s_, 35,60,-20},
//         {start_epoch_ +  6*dwell_s_, 35,20,-20},
//         {start_epoch_ +  7*dwell_s_, 35,60,-20},
//       };
//     }
//     if (known_sub2_.empty()){
//       known_sub2_ = {
//         {start_epoch_ +  1*dwell_s_, 35,20,-20},
//         {start_epoch_ +  2*dwell_s_, 35,60,-20},
//         {start_epoch_ +  6*dwell_s_, 35,20,-20},
//         {start_epoch_ +  7*dwell_s_, 35,60,-20},
//       };
//     }

//     // Mirror into per-identity for current holders (so identity snapshots are populated from the start)
//     const auto sch = compute(now_unix());
//     int64_t id_mid  = identityOfTeamId(sch.id_mid);
//     int64_t id_sub1 = identityOfTeamId(sch.id_s1);
//     int64_t id_sub2 = identityOfTeamId(sch.id_s2);
//     if (id_mid  >= 0) mergeIntoCount(per_identity_plans_[id_mid],  known_mid_);
//     if (id_sub1 >= 0) mergeIntoCount(per_identity_plans_[id_sub1], known_sub1_);
//     if (id_sub2 >= 0) mergeIntoCount(per_identity_plans_[id_sub2], known_sub2_);
//   }

//   // state / last-event info
//   std::string last_local_event_type_;  // e.g., "foreign_uuv", "pipeline_break", "low_battery"

//   // state info
//   std::string sub_id_;
//   std::vector<std::string> team_ids_;
//   std::string leader_id_;
//   int    leader0_idx_{0};
//   double start_epoch_{0.0};
//   double dwell_s_{20.0};
//   bool   publish_current_pose_{false};

//   int  last_slot5_{-1};
//   std::string last_mid_id_;
//   bool published_this_slot_{false};

//   // Plans (role-based, for expected poses and legacy pubs)
//   PlanVec my_plan_;
//   PlanVec my_prev_plan_;
//   PlanVec known_sub1_;
//   PlanVec known_sub2_;
//   PlanVec known_mid_;
//   PlanVec known_mid_prev_;

//   // Per-identity plan/history store (history == plans; sanitized on serialization)
//   std::unordered_map<int64_t, PlanVec> per_identity_plans_;

//   // New plan points accumulated since last surface (for SURFACE_RELAY reporting)
//   std::size_t added_since_surface_sub1_{0};
//   std::size_t added_since_surface_sub2_{0};
//   std::size_t added_since_surface_mid_{0};

//   // Latest events per role + counters since last surface
//   std::unordered_map<std::string, custom_msg::msg::Event> latest_events_;
//   std::unordered_map<std::string, std::size_t> events_since_surface_{{"mid",0},{"sub1",0},{"sub2",0}};

//   // Publishers
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr plans_pub_sub1_;
//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr plans_pub_sub2_;
//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr plans_pub_mid_;

//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr echo_pub_sub1_;
//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr echo_pub_sub2_;
//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr echo_pub_mid_;

//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr relay_pub_sub1_;
//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr relay_pub_sub2_;
//   rclcpp::Publisher<custom_msg::msg::Plan>::SharedPtr relay_pub_mid_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr relay_snapshot_json_pub_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr relay_events_json_pub_;

//   rclcpp::Publisher<custom_msg::msg::Event>::SharedPtr event_pub_sub1_;
//   rclcpp::Publisher<custom_msg::msg::Event>::SharedPtr event_pub_sub2_;
//   rclcpp::Publisher<custom_msg::msg::Event>::SharedPtr event_pub_mid_;

//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exp_pub_sub1_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exp_pub_sub2_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exp_pub_mid_;

//   // Subscribers
//   rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr plans_sub_s1_;
//   rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr plans_sub_s2_;
//   rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr plans_sub_mid_;
//   rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr echo_sub_s1_;
//   rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr echo_sub_s2_;
//   rclcpp::Subscription<custom_msg::msg::Plan>::SharedPtr echo_sub_mid_;

//   rclcpp::Subscription<custom_msg::msg::Event>::SharedPtr event_sub_s1_;
//   rclcpp::Subscription<custom_msg::msg::Event>::SharedPtr event_sub_s2_;
//   rclcpp::Subscription<custom_msg::msg::Event>::SharedPtr event_sub_mid_;

//   // Timer
//   rclcpp::TimerBase::SharedPtr timer_;
//   int last_log_second_{-1};
// };

// int main(int argc, char **argv){
//   rclcpp::init(argc, argv);
//   auto boot = std::make_shared<rclcpp::Node>("bootstrap");
//   boot->declare_parameter<std::string>("sub_id");
//   const std::string sub_id = boot->get_parameter("sub_id").as_string();
//   boot.reset();

//   auto node = std::make_shared<SubNode>(sub_id);
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
  // }
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
#include <cstdlib>  // for std::atof

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

// JSON
#include <nlohmann/json.hpp>
using nlohmann::json;

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

// Convert a PlanVec into custom_msg/Plan, with explicit SUBJECT team_id and ROLE tag.
// - out.header.frame_id  := subject_team_id  (WHO this plan is about; identity anchor)
// - each path/header.frame_id := role tag (e.g., "mid","sub1","sub2","relay") for legacy consumers
static custom_msg::msg::Plan toPlanMsgSubject(const PlanVec& plan,
                                              const std::string& subject_team_id,
                                              const std::string& role_tag)
{
  custom_msg::msg::Plan out;
  out.header.frame_id = subject_team_id;      // <<< identity/subject!
  out.header.stamp = toStamp(now_unix());

  nav_msgs::msg::Path path;
  path.header.frame_id = role_tag;            // legacy: role context
  path.header.stamp = out.header.stamp;

  path.poses.reserve(plan.size());
  for(const auto& w: plan){
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = role_tag;
    ps.header.stamp = toStamp(w.t);
    ps.pose.position.x = w.x;
    ps.pose.position.y = w.y;
    ps.pose.position.z = w.z;
    ps.pose.orientation.w = 1.0;
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

// merge add into base; return number of newly added points (exact t,x,y,z match)
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
    if (w.t <= last_t) w.t = last_t + eps; // enforce strictly increasing time
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

// ===================== NEW: Robust "plan from array" helpers =====================
static double num_from_json(const json& v, double def=0.0){
  if (v.is_number_float()) return v.get<double>();
  if (v.is_number_integer()) return static_cast<double>(v.get<long long>());
  if (v.is_string()) {
    try { return std::stod(v.get<std::string>()); } catch(...) { return def; }
  }
  return def;
}

static void accumulate_waypoints_from_json_array(const json& arr, PlanVec& out){
  for (const auto& e : arr){
    if (e.is_object()){
      const double t = num_from_json(e.value("t", 0.0));
      const double x = num_from_json(e.value("x", 0.0));
      const double y = num_from_json(e.value("y", 0.0));
      const double z = num_from_json(e.value("z", 0.0));
      out.push_back({t,x,y,z});
    } else if (e.is_array()){
      // support [t,x,y,z] or [x,y,z] (auto time)
      double t=0,x=0,y=0,z=0;
      if (e.size() >= 4){ t=num_from_json(e[0]); x=num_from_json(e[1]); y=num_from_json(e[2]); z=num_from_json(e[3]); }
      else if (e.size() == 3){ x=num_from_json(e[0]); y=num_from_json(e[1]); z=num_from_json(e[2]); }
      out.push_back({t,x,y,z});
    }
  }
}

// Try JSON first; fallback to "t,x,y,z ; t,x,y,z ..."
static PlanVec planFromArrayString(const std::string& raw){
  PlanVec v;

  // --- JSON path ---
  try{
    json j = json::parse(raw);

    if (j.is_array()){
      accumulate_waypoints_from_json_array(j, v);
    } else if (j.is_object()){
      if (j.contains("poses") && j["poses"].is_array()){
        accumulate_waypoints_from_json_array(j["poses"], v);
      } else if (j.contains("paths") && j["paths"].is_array() && !j["paths"].empty()){
        const auto& p0 = j["paths"].front();
        if (p0.contains("poses") && p0["poses"].is_array()){
          accumulate_waypoints_from_json_array(p0["poses"], v);
        }
      }
    }
  } catch(...){
    // ignore and try CSV fallback
  }

  // --- CSV/whitespace fallback ---
  if (v.empty()){
    std::vector<double> nums;
    nums.reserve(64);
    std::string tok;
    for (char c : raw){
      if (std::isspace(static_cast<unsigned char>(c)) || c==',' || c==';' || c=='[' || c==']' || c=='{'
          || c=='}' || c=='"'){
        if (!tok.empty()){
          try { nums.push_back(std::stod(tok)); } catch(...) { /* skip */ }
          tok.clear();
        }
      } else {
        tok.push_back(c);
      }
    }
    if (!tok.empty()){
      try { nums.push_back(std::stod(tok)); } catch(...) {}
    }
    for (size_t i=0; i+3<nums.size(); i+=4){
      v.push_back({nums[i+0], nums[i+1], nums[i+2], nums[i+3]});
    }
  }

  // If some entries had t==0 because array form omitted time, auto-increment from now.
  if (!v.empty()){
    bool missing_times = std::all_of(v.begin(), v.end(), [](const Waypoint& w){ return std::fabs(w.t) < 1e-12; });
    if (missing_times){
      double t0 = now_unix();
      const double dt = 1.0; // 1s spacing
      for (size_t i=0;i<v.size();++i) v[i].t = t0 + dt*static_cast<double>(i);
    }
  }

  // Final tidy
  sanitizePlanHistory(v, 30.0);
  std::sort(v.begin(), v.end(), [](const Waypoint& a, const Waypoint& b){ return a.t < b.t; });
  return v;
}
// =================== END: Robust "plan from array" helpers ======================

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
    this->declare_parameter<std::vector<int64_t>>("team_identity_numbers", {0,1,2});

    Identifier = this->get_parameter("Identifier").as_int();
    team_ids_            = this->get_parameter("team_ids").as_string_array();
    leader_id_           = this->get_parameter("leader_id").as_string();
    start_epoch_         = this->get_parameter("start_epoch").as_double();
    dwell_s_             = this->get_parameter("state_dwell_s").as_double();
    publish_current_pose_= this->get_parameter("publish_current_pose").as_bool();
    team_identity_numbers_ = this->get_parameter("team_identity_numbers").as_integer_array();

    if (std::find(team_ids_.begin(), team_ids_.end(), sub_id_) == team_ids_.end())
      team_ids_.push_back(sub_id_);
    { // dedupe preserving first occurrence
      std::vector<std::string> uniq; uniq.reserve(team_ids_.size());
      for (auto& s: team_ids_) if (std::find(uniq.begin(), uniq.end(), s)==uniq.end()) uniq.push_back(s);
      team_ids_.swap(uniq);
    }
    if (team_identity_numbers_.size() != team_ids_.size()){
      team_identity_numbers_.resize(team_ids_.size());
      for (size_t i=0;i<team_ids_.size();++i) team_identity_numbers_[i] = static_cast<int64_t>(i);
    }
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

    // Plan pubs (role topics kept for compatibility)
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

    // subscriptions: Plans (role topics)
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

    // --- NEW: command topic to trigger events without services ---
    event_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/events/command", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg){
        this->onEventCommand(*msg);
      });

    // --- NEW: Accept raw array/JSON to produce a Plan ---
    plan_array_sub_ = create_subscription<std_msgs::msg::String>(
      "/plans/from_array", 10,
      [this](std_msgs::msg::String::ConstSharedPtr s){
        this->onPlanArray(*s);
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
    std::string id_mid, id_s1, id_s2;
    int slot5;
  };

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
    if (sub_id_==mid) r=Role::MID_TIER;
    else if (sub_id_==s1) r=Role::SUB1;
    else if (sub_id_==s2) r=Role::SUB2;

    return Sched{r,ms,ss,mid,s1,s2,slot5};
  }

  // helpers
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
    ev.header.frame_id = sub_id_; // identity/subject of this event
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

  // --- NEW: simple "k=v,k=v" parser for command topic ---
  static std::unordered_map<std::string,std::string> kvparse(const std::string& s){
    std::unordered_map<std::string,std::string> m;
    std::string key, val; bool in_key=true;
    auto flush=[&](){ if(!key.empty()){ m[key]=val; key.clear(); val.clear(); in_key=true; } };
    for(char c: s){
      if (in_key){
        if (c=='=') in_key=false;
        else if (c==',' ) { /* ignore stray */ }
        else key.push_back(c);
      } else {
        if (c==',') { flush(); }
        else val.push_back(c);
      }
    }
    flush();
    return m;
  }

  // --- NEW: command handler + state for pending events ---
  struct PendingEvent {
    bool has{false};
    std::string type{"manual_event"};
    bool has_pos{false};
    geometry_msgs::msg::Point pos{};
    std::string role_override{}; // "", "sub1", "sub2", "mid"
    bool publish_now{true};      // if false, hold until SURFACE_INIT
  } pending_event_;

  void onEventCommand(const std_msgs::msg::String& msg){
    const auto m = kvparse(msg.data);

    PendingEvent pe;
    pe.type = "manual_event";
    pe.publish_now = true;
    pe.has_pos = false;
    pe.role_override.clear();

    if (auto it=m.find("event_type"); it!=m.end()) pe.type = it->second;
    if (auto it=m.find("role"); it!=m.end()) {
      const auto& r = it->second;
      if (r=="sub1"||r=="sub2"||r=="mid") pe.role_override = r;
    }
    if (auto it=m.find("publish_now"); it!=m.end()) {
      const std::string& v = it->second;
      pe.publish_now = (v=="1"||v=="true"||v=="TRUE"||v=="True");
    }
    bool gotx=false, goty=false, gotz=false;
    if (auto it=m.find("x"); it!=m.end()) { pe.pos.x = std::atof(it->second.c_str()); gotx=true; }
    if (auto it=m.find("y"); it!=m.end()) { pe.pos.y = std::atof(it->second.c_str()); goty=true; }
    if (auto it=m.find("z"); it!=m.end()) { pe.pos.z = std::atof(it->second.c_str()); gotz=true; }
    pe.has_pos = (gotx && goty && gotz);

    pending_event_ = pe;
    pending_event_.has = true;

    RCLCPP_INFO(get_logger(),
      "[%s] queued event trigger: type='%s' role_override='%s' publish_now=%d pos:%s",
      sub_id_.c_str(), pe.type.c_str(),
      pe.role_override.c_str(), pe.publish_now ? 1:0,
      pe.has_pos ? "yes":"no");
  }

  void publishEventToRole(const std::string& role_key,
                          const std::string& ev_type,
                          const geometry_msgs::msg::Point* pos_opt /*nullable*/)
  {
    custom_msg::msg::Event ev;
    ev.header.stamp = toStamp(now_unix());
    ev.header.frame_id = sub_id_;
    ev.event_type = ev_type;

    geometry_msgs::msg::PoseStamped ps;
    ps.header = ev.header;

    if (pos_opt){
      ps.pose.position = *pos_opt;
    } else {
      const auto sch = compute(now_unix());
      if (role_key=="mid")      ps.pose.position = midTarget(sch.mid_state);
      else if (role_key=="sub1")ps.pose.position = sub1Target(sch.sub_state);
      else                      ps.pose.position = sub2Target(sch.sub_state);
    }
    ps.pose.orientation.w = 1.0;

    ev.waypoints.clear();
    ev.waypoints.push_back(ps);

    if (role_key=="mid")      event_pub_mid_->publish(ev);
    else if (role_key=="sub1")event_pub_sub1_->publish(ev);
    else                      event_pub_sub2_->publish(ev);

    RCLCPP_INFO(get_logger(), "[%s] EVENT PUBLISHED on /events/%s: %s (x=%.2f y=%.2f z=%.2f)",
      sub_id_.c_str(), role_key.c_str(), ev.event_type.c_str(),
      ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
  }

  // ============ NEW: accept JSON/array plan and publish as proper Plan ==========
  void onPlanArray(const std_msgs::msg::String& s){
    // Parse incoming array into waypoints
    PlanVec pv = planFromArrayString(s.data);
    if (pv.empty()){
      RCLCPP_WARN(get_logger(), "[%s] /plans/from_array: no waypoints parsed", sub_id_.c_str());
      return;
    }

    // Decide subject + role tags based on current schedule
    const auto sch = compute(now_unix());
    const std::string role_now =
        (sch.my_role==Role::MID_TIER? "mid" : (sch.my_role==Role::SUB1? "sub1" : "sub2"));
    const std::string subject_team_id =
        (role_now=="mid") ? sch.id_mid : (role_now=="sub1" ? sch.id_s1 : sch.id_s2);

    // Publish as a proper Plan message on the role topic (for legacy consumers)
    auto msg = toPlanMsgSubject(pv, subject_team_id, role_now);
    if (role_now=="mid")      plans_pub_mid_->publish(msg);
    else if (role_now=="sub1")plans_pub_sub1_->publish(msg);
    else                      plans_pub_sub2_->publish(msg);

    // Merge into this node's knowledge stores just like onPlanTopic()
    std::size_t added = 0;
    if (role_now=="sub1"){ added = mergeIntoCount(known_sub1_, pv); added_since_surface_sub1_ += added; }
    else if (role_now=="sub2"){ added = mergeIntoCount(known_sub2_, pv); added_since_surface_sub2_ += added; }
    else { added = mergeIntoCount(known_mid_, pv); added_since_surface_mid_ += added; }

    int64_t idn = identityOfTeamId(subject_team_id);
    if (idn >= 0) mergeIntoCount(per_identity_plans_[idn], pv);

    // Bookkeeping for "my plan" if applicable
    const std::string my_key = roleKeyNow();
    if (role_now == my_key){
      if (!my_prev_plan_.empty()){
        auto echo_pa = toPlanMsgSubject(my_prev_plan_, sub_id_, my_key + std::string("_prev"));
        echo_pub_for_key(my_key)->publish(echo_pa);
      }
      my_prev_plan_ = my_plan_;
      my_plan_ = pv;
    }

    RCLCPP_INFO(get_logger(), "[%s] /plans/from_array parsed %zu pts → role=%s (subject=%s) [+%zu merged]",
                sub_id_.c_str(), pv.size(), role_now.c_str(), subject_team_id.c_str(), added);
  }
  // ============================================================================

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

    // --- NEW: handle queued event trigger (no services) ---
    if (pending_event_.has){
      // decide which role's topic to use
      std::string role_out = pending_event_.role_override;
      if (role_out.empty()){
        role_out = (sch.my_role==Role::MID_TIER) ? "mid" : (sch.my_role==Role::SUB1 ? "sub1" : "sub2");
      }
      const bool should_publish_now = pending_event_.publish_now ||
                                      (sch.mid_state == MidState::SURFACE_INIT);
      if (should_publish_now){
        const geometry_msgs::msg::Point* pos_ptr = pending_event_.has_pos ? &pending_event_.pos : nullptr;
        publishEventToRole(role_out, pending_event_.type, pos_ptr);
        pending_event_.has = false; // clear
      }
    }

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

    // At SURFACE_INIT, non-MID subs publish a one-shot event
    if (sch.mid_state == MidState::SURFACE_INIT) {
      publishMyEventIfNeeded(sch);
    }

    if (sch.my_role == Role::MID_TIER) {
      switch (sch.mid_state) {
        case MidState::SURFACE_INIT:
          RCLCPP_INFO(get_logger(),
            "[%s] MID SURFACE_INIT: uploading known plans (s1=%zu, s2=%zu, mid=%zu);",
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
          RCLCPP_INFO(get_logger(), "[%s] MID RETURN_END:", sub_id_.c_str());
          break;
      }
    }
  }

  // MID: one-shot handoff publisher in the slot
  void midHandoffOnce(const std::string& role_key, const std::string& recipient_team_id){
    if (published_this_slot_) return;
    published_this_slot_ = true;

    RCLCPP_INFO(get_logger(),
      "[%s] MID handoff → %s (role=%s): publishing /plans/%s and /plans/mid",
      sub_id_.c_str(), recipient_team_id.c_str(), role_key.c_str(), role_key.c_str());

    // Subject team IDs: the plans we place on each role topic are ABOUT the current holder of that role
    const auto sch = compute(now_unix());
    const std::string sub1_subject = sch.id_s1;
    const std::string sub2_subject = sch.id_s2;
    const std::string mid_subject  = sch.id_mid;

    if (role_key == "sub1") {
      plans_pub_sub1_->publish(toPlanMsgSubject(known_sub1_, sub1_subject, "mid"));
    } else {
      plans_pub_sub2_->publish(toPlanMsgSubject(known_sub2_, sub2_subject, "mid"));
    }
    plans_pub_mid_->publish (toPlanMsgSubject(known_mid_, mid_subject, "mid"));

    if (!known_mid_prev_.empty()) {
      auto pa = toPlanMsgSubject(known_mid_prev_, mid_subject, "mid_prev");
      echo_pub_mid_->publish(pa);
    }
    known_mid_prev_ = known_mid_;
  }

  // ROLE-topic plan reception → merge role knowledge + per-identity store
  void onPlanTopic(const custom_msg::msg::Plan& pm, const std::string& topic_key /* "sub1"|"sub2"|"mid" */) {
    const std::string my_key = roleKeyNow();
    PlanVec new_plan = fromPlanMsg(pm);

    // Merge into role-based knowledge (kept for expected poses + legacy pubs)
    std::size_t added = 0;
    if (topic_key=="sub1")      { added = mergeIntoCount(known_sub1_, new_plan); added_since_surface_sub1_ += added; }
    else if (topic_key=="sub2") { added = mergeIntoCount(known_sub2_, new_plan); added_since_surface_sub2_ += added; }
    else                        { added = mergeIntoCount(known_mid_,  new_plan); added_since_surface_mid_  += added; }

    // Determine SUBJECT team_id from header.frame_id (preferred). Fallback: current holder of the role.
    std::string subject_team_id = pm.header.frame_id;
    if (idnum_by_teamid_.find(subject_team_id) == idnum_by_teamid_.end()){
      const auto sch = compute(now_unix());
      subject_team_id = (topic_key=="mid")? sch.id_mid : (topic_key=="sub1"? sch.id_s1 : sch.id_s2);
    }
    int64_t idnum = identityOfTeamId(subject_team_id);
    if (idnum >= 0){
      mergeIntoCount(per_identity_plans_[idnum], new_plan);
    }

    // If this plan is mine-in-my-current-role, handle echo bookkeeping
    if (topic_key == my_key) {
      if (!my_prev_plan_.empty()) {
        auto echo_pa = toPlanMsgSubject(my_prev_plan_, sub_id_, my_key + std::string("_prev"));
        echo_pub_for_key(my_key)->publish(echo_pa);
        RCLCPP_INFO(get_logger(), "[%s] ECHO %s prev plan (%zu pts)", sub_id_.c_str(), my_key.c_str(), my_prev_plan_.size());
      }
      my_prev_plan_ = my_plan_;
      my_plan_ = std::move(new_plan);
      RCLCPP_INFO(get_logger(), "[%s] MY PLAN updated for %s (%zu pts, +%zu new merged)", sub_id_.c_str(), my_key.c_str(), my_plan_.size(), added);
    }
  }

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
    // publish role-based merged knowledge (legacy)
    const auto sch = compute(now_unix());
    const std::string sub1_subject = sch.id_s1;
    const std::string sub2_subject = sch.id_s2;
    const std::string mid_subject  = sch.id_mid;

    relay_pub_sub1_->publish(toPlanMsgSubject(known_sub1_, sub1_subject, "relay"));
    relay_pub_sub2_->publish(toPlanMsgSubject(known_sub2_, sub2_subject, "relay"));
    relay_pub_mid_->publish (toPlanMsgSubject(known_mid_,  mid_subject,  "relay"));

    // Ensure we have per-identity entries
    for (const auto& team_id : team_ids_){
      int64_t idn = identityOfTeamId(team_id);
      (void) per_identity_plans_[idn];
    }

    // Sanitize copies
    std::unordered_map<int64_t, PlanVec> by_id_sanitized;
    for (const auto& team_id : team_ids_){
      const int64_t idn = identityOfTeamId(team_id);
      PlanVec cp = per_identity_plans_[idn];
      sanitizePlanHistory(cp, 30.0);
      by_id_sanitized[idn] = std::move(cp);
    }

    // Also role-based sanitized (compat)
    PlanVec k_mid = known_mid_, k_s1 = known_sub1_, k_s2 = known_sub2_;
    sanitizePlanHistory(k_mid, 30.0);
    sanitizePlanHistory(k_s1,  30.0);
    sanitizePlanHistory(k_s2,  30.0);

    // Build JSON (identity-first)
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

    for (size_t i=0;i<team_ids_.size(); ++i){
      const std::string& team_id = team_ids_[i];
      const int64_t idn = team_identity_numbers_[i];
      const std::string role_now = roleOfTeamId(team_id, sch);
      const PlanVec& p  = by_id_sanitized[idn];
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
      "[%s] SURFACE_RELAY | identity-first snapshot published", sub_id_.c_str());
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

  // Seed some plans so handoffs have content + seed per-identity for current holders
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
    // Mirror into per-identity for current holders
    const auto sch = compute(now_unix());
    int64_t id_mid  = identityOfTeamId(sch.id_mid);
    int64_t id_sub1 = identityOfTeamId(sch.id_s1);
    int64_t id_sub2 = identityOfTeamId(sch.id_s2);
    if (id_mid  >= 0) mergeIntoCount(per_identity_plans_[id_mid],  known_mid_);
    if (id_sub1 >= 0) mergeIntoCount(per_identity_plans_[id_sub1], known_sub1_);
    if (id_sub2 >= 0) mergeIntoCount(per_identity_plans_[id_sub2], known_sub2_);
  }

  // state / last-event info
  std::string last_local_event_type_;

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

  // Plans (role-based)
  PlanVec my_plan_;
  PlanVec my_prev_plan_;
  PlanVec known_sub1_;
  PlanVec known_sub2_;
  PlanVec known_mid_;
  PlanVec known_mid_prev_;

  // Per-identity plan/history store (history == plan)
  std::unordered_map<int64_t, PlanVec> per_identity_plans_;

  // New plan points accumulated since last surface
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

  // NEW: command subscriber + array plan subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_array_sub_;

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
