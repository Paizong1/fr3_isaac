#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <boost/bind/bind.hpp>
#include <ignition/math/Pose3.hh>
#include <algorithm>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace gazebo
{
class GraspFixPlugin final : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    if (!model)
    {
      gzerr << "GraspFixPlugin: model is null\n";
      return;
    }
    this->model_ = std::move(model);
    this->world_ = this->model_->GetWorld();
    if (!this->world_)
    {
      gzerr << "GraspFixPlugin: world is null\n";
      return;
    }

    if (sdf && sdf->HasElement("disable_collisions_on_attach"))
    {
      this->disable_collisions_on_attach_ = sdf->Get<bool>("disable_collisions_on_attach");
    }
    if (sdf && sdf->HasElement("max_grip_count"))
    {
      this->max_grip_count_ = std::max(1, sdf->Get<int>("max_grip_count"));
    }
    if (sdf && sdf->HasElement("grip_count_threshold"))
    {
      this->grip_count_threshold_ = std::max(1, sdf->Get<int>("grip_count_threshold"));
    }
    if (sdf && sdf->HasElement("update_rate"))
    {
      const int rate = std::max(1, sdf->Get<int>("update_rate"));
      this->update_period_ = common::Time(1.0 / static_cast<double>(rate));
    }
    if (sdf && sdf->HasElement("min_gripping_contacts"))
    {
      this->min_gripping_contacts_ = std::max(1, sdf->Get<int>("min_gripping_contacts"));
    }
    if (sdf && sdf->HasElement("grasp_object_substring"))
    {
      this->grasp_object_substring_ = sdf->Get<std::string>("grasp_object_substring");
    }

    sdf::ElementPtr arm = sdf ? sdf->GetElement("arm") : sdf::ElementPtr();
    if (!arm)
    {
      gzerr << "GraspFixPlugin: missing <arm>\n";
      return;
    }

    if (!arm->HasElement("palm_link"))
    {
      gzerr << "GraspFixPlugin: missing <palm_link>\n";
      return;
    }
    this->palm_link_name_ = arm->Get<std::string>("palm_link");
    this->palm_link_ = this->model_->GetLink(this->palm_link_name_);
    if (!this->palm_link_)
    {
      gzerr << "GraspFixPlugin: palm link not found: " << this->palm_link_name_ << "\n";
      return;
    }

    std::vector<std::string> collision_names;
    int gripper_links_with_collisions = 0;
    int gripper_link_index = 0;
    for (sdf::ElementPtr gl = arm->GetElement("gripper_link"); gl; gl = gl->GetNextElement("gripper_link"))
    {
      const std::string link_name = gl->Get<std::string>();
      auto link = this->model_->GetLink(link_name);
      if (!link)
      {
        gzwarn << "GraspFixPlugin: gripper link not found: " << link_name << "\n";
        continue;
      }
      bool link_has_collision = false;
      for (unsigned int i = 0;; ++i)
      {
        auto collision = link->GetCollision(i);
        if (!collision)
        {
          break;
        }
        const std::string scoped = collision->GetScopedName();
        this->RegisterGripperCollision(scoped, gripper_link_index);
        collision_names.push_back(scoped);
        link_has_collision = true;
        gzmsg << "GraspFixPlugin: tracking collision [" << scoped << "]\n";
      }
      if (link_has_collision)
      {
        ++gripper_links_with_collisions;
      }
      ++gripper_link_index;
    }

    if (gripper_links_with_collisions < 2 || collision_names.empty())
    {
      gzerr << "GraspFixPlugin: need at least 2 gripper links with collisions\n";
      return;
    }

    gzmsg << "GraspFixPlugin: loaded on model [" << this->model_->GetName() << "] min_contacts="
          << this->min_gripping_contacts_ << " object_filter=[" << this->grasp_object_substring_
          << "] gripper_links=" << gripper_links_with_collisions
          << " tracked_collisions=" << collision_names.size() << std::endl;

    this->node_ = transport::NodePtr(new transport::Node());
    this->node_->Init(this->world_->Name());

    auto physics = this->world_->Physics();
    if (!physics)
    {
      gzerr << "GraspFixPlugin: physics is null\n";
      return;
    }
    auto contact_manager = physics->GetContactManager();
    if (!contact_manager)
    {
      gzerr << "GraspFixPlugin: contact manager is null\n";
      return;
    }
    contact_manager->PublishContacts();

    // Use the world contact stream and filter in OnUpdate. CreateFilter() is brittle across Gazebo builds.
    this->contact_sub_ =
      this->node_->Subscribe("~/physics/contacts", &GraspFixPlugin::OnContact, this);
    gzmsg << "GraspFixPlugin: subscribed to ~/physics/contacts\n";
    this->last_update_wall_ = common::Time::GetWallTime();
    this->update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&GraspFixPlugin::OnUpdate, this));
  }

private:
  static bool NamesMatch(const std::string & a, const std::string & b)
  {
    if (a == b)
    {
      return true;
    }
    if (a.size() >= b.size() && a.compare(a.size() - b.size(), b.size(), b) == 0)
    {
      return true;
    }
    if (b.size() >= a.size() && b.compare(b.size() - a.size(), a.size(), a) == 0)
    {
      return true;
    }
    return false;
  }

  void RegisterGripperCollision(const std::string & scoped_name, int gripper_index)
  {
    this->collision_to_gripper_index_[scoped_name] = gripper_index;
    const auto pos = scoped_name.rfind("::");
    if (pos != std::string::npos)
    {
      this->collision_to_gripper_index_[scoped_name.substr(pos + 2)] = gripper_index;
    }
  }

  int LookupGripperIndex(const std::string & collision_name) const
  {
    const auto direct = this->collision_to_gripper_index_.find(collision_name);
    if (direct != this->collision_to_gripper_index_.end())
    {
      return direct->second;
    }
    for (const auto & kv : this->collision_to_gripper_index_)
    {
      if (NamesMatch(collision_name, kv.first))
      {
        return kv.second;
      }
    }
    return -1;
  }

  physics::CollisionPtr FindCollisionByName(const std::string & collision_name) const
  {
    if (!this->world_)
    {
      return physics::CollisionPtr();
    }
    auto ent = this->world_->EntityByName(collision_name);
    auto collision = boost::dynamic_pointer_cast<physics::Collision>(ent);
    if (collision)
    {
      return collision;
    }
    for (const auto & model : this->world_->Models())
    {
      if (!model)
      {
        continue;
      }
      const auto & links = model->GetLinks();
      for (const auto & link : links)
      {
        if (!link)
        {
          continue;
        }
        for (unsigned int ci = 0;; ++ci)
        {
          auto col = link->GetCollision(ci);
          if (!col)
          {
            break;
          }
          if (NamesMatch(col->GetScopedName(), collision_name))
          {
            return col;
          }
        }
      }
    }
    return physics::CollisionPtr();
  }

  void OnContact(ConstContactsPtr &msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->contacts_ = *msg;
    this->has_contacts_ = true;
  }

  bool IsValidGraspTarget(const std::string & collision_name) const
  {
    if (!this->grasp_object_substring_.empty() &&
        collision_name.find(this->grasp_object_substring_) == std::string::npos)
    {
      return false;
    }
    static const char * k_exclude[] = {"cafe_table", "ground_plane", "ground::", "::ground"};
    for (const char * ex : k_exclude)
    {
      if (collision_name.find(ex) != std::string::npos)
      {
        return false;
      }
    }
    return true;
  }

  void OnUpdate()
  {
    const auto now = common::Time::GetWallTime();
    if ((now - this->last_update_wall_) < this->update_period_)
    {
      return;
    }
    this->last_update_wall_ = now;

    gazebo::msgs::Contacts contacts;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (!this->has_contacts_)
      {
        return;
      }
      contacts = this->contacts_;
    }

    std::unordered_map<std::string, std::unordered_set<int>> obj_to_grippers;
    obj_to_grippers.reserve(static_cast<size_t>(contacts.contact_size()));
    for (int i = 0; i < contacts.contact_size(); ++i)
    {
      const auto &c = contacts.contact(i);
      const std::string &a = c.collision1();
      const std::string &b = c.collision2();
      const int ia = this->LookupGripperIndex(a);
      const int ib = this->LookupGripperIndex(b);
      if (ia >= 0 && ib < 0)
      {
        if (this->IsValidGraspTarget(b))
        {
          obj_to_grippers[b].insert(ia);
        }
      }
      else if (ib >= 0 && ia < 0)
      {
        if (this->IsValidGraspTarget(a))
        {
          obj_to_grippers[a].insert(ib);
        }
      }
    }

    std::string best_obj;
    size_t best_cnt = 0;
    for (auto &kv : obj_to_grippers)
    {
      if (kv.second.size() > best_cnt)
      {
        best_cnt = kv.second.size();
        best_obj = kv.first;
      }
    }

    const bool gripping_now = best_cnt >= static_cast<size_t>(this->min_gripping_contacts_);
    if (this->attach_joint_)
    {
      // Latched grasp: after attach we disable banana collisions, so contact reports stop.
      // Do not decay grip_counter or auto-detach while the fixed joint is active.
      this->grip_counter_ = this->max_grip_count_;
    }
    else if (gripping_now)
    {
      if (this->current_obj_collision_.empty() || this->current_obj_collision_ == best_obj)
      {
        this->current_obj_collision_ = best_obj;
        this->grip_counter_ = std::min(this->max_grip_count_, this->grip_counter_ + 1);
      }
      else
      {
        this->Detach();
        this->current_obj_collision_ = best_obj;
        this->grip_counter_ = 1;
      }
    }
    else
    {
      this->grip_counter_ = std::max(0, this->grip_counter_ - 1);
      if (this->grip_counter_ == 0)
      {
        this->current_obj_collision_.clear();
      }
    }

    if (this->grip_counter_ >= this->grip_count_threshold_ && !this->attach_joint_ &&
        !this->current_obj_collision_.empty())
    {
      this->AttachToCollision(this->current_obj_collision_);
    }

    if (best_cnt > 0 && !this->attach_joint_ && ++this->contact_log_throttle_ % 50 == 1)
    {
      gzmsg << "GraspFixPlugin: contact with [" << best_obj << "] fingers=" << best_cnt
            << " grip_counter=" << this->grip_counter_ << std::endl;
    }
  }

  void AttachToCollision(const std::string &collision_name)
  {
    if (this->attach_joint_)
    {
      return;
    }
    if (!this->world_ || !this->palm_link_)
    {
      return;
    }
    auto collision = this->FindCollisionByName(collision_name);
    if (!collision)
    {
      gzwarn << "GraspFixPlugin: attach failed, collision not found [" << collision_name << "]\n";
      return;
    }
    auto obj_link = collision->GetLink();
    if (!obj_link)
    {
      gzwarn << "GraspFixPlugin: attach failed, link missing for [" << collision_name << "]\n";
      return;
    }
    auto physics = this->world_->Physics();
    if (!physics)
    {
      return;
    }

    const ignition::math::Pose3d diff = obj_link->WorldPose() - this->palm_link_->WorldPose();
    this->attach_joint_ = physics->CreateJoint("fixed", this->model_);
    if (!this->attach_joint_)
    {
      return;
    }
    this->attach_joint_->Load(this->palm_link_, obj_link, diff);
    this->attach_joint_->Init();
    this->attached_obj_link_ = obj_link;
    gzmsg << "GraspFixPlugin: attached to [" << collision_name << "] link [" << obj_link->GetName()
          << "]" << std::endl;
    if (this->disable_collisions_on_attach_)
    {
      obj_link->SetCollideMode("none");
      this->collisions_disabled_ = true;
    }
  }

  void Detach()
  {
    if (this->attach_joint_)
    {
      if (this->collisions_disabled_ && this->attached_obj_link_)
      {
        this->attached_obj_link_->SetCollideMode("all");
      }
      this->attach_joint_->Detach();
      this->attach_joint_.reset();
    }
    this->attached_obj_link_.reset();
    this->collisions_disabled_ = false;
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  transport::NodePtr node_;
  transport::SubscriberPtr contact_sub_;
  event::ConnectionPtr update_connection_;

  std::string palm_link_name_;
  physics::LinkPtr palm_link_;

  std::mutex mutex_;
  gazebo::msgs::Contacts contacts_;
  bool has_contacts_{false};

  common::Time last_update_wall_{0, 0};
  common::Time update_period_{0.5};

  int grip_count_threshold_{1};
  int max_grip_count_{2};
  int min_gripping_contacts_{1};
  std::string grasp_object_substring_{"banana"};
  bool disable_collisions_on_attach_{true};

  std::unordered_map<std::string, int> collision_to_gripper_index_;
  std::string current_obj_collision_;
  int grip_counter_{0};
  int contact_log_throttle_{0};

  physics::JointPtr attach_joint_;
  physics::LinkPtr attached_obj_link_;
  bool collisions_disabled_{false};
};

GZ_REGISTER_MODEL_PLUGIN(GraspFixPlugin)
}  // namespace gazebo
