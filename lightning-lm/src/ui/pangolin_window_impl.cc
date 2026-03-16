#include <pangolin/display/default_font.h>
#include <string>
#include <thread>

#include "common/options.h"
#include "common/std_types.h"
#include "core/lightning_math.hpp"
#include "ui/pangolin_window_impl.h"

namespace lightning::ui {

bool PangolinWindowImpl::Init() {
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(win_name_, win_width_, win_height_);

    // 3D mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // opengl buffer
    AllocateBuffer();

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();

    // 雷达定位轨迹opengl设置
    traj_newest_state_.reset(new ui::UiTrajectory(Vec3f(1.0, 0.0, 0.0)));  // 红色
    traj_scans_.reset(new ui::UiTrajectory(Vec3f(0.0, 1.0, 0.0)));         // 绿色

    current_scan_.reset(new PointCloudType);  // 重置pcl点云指针
    current_scan_ui_.reset(new ui::UiCloud);  // 重置用于渲染的点云指针

    /// data log
    log_vel_.SetLabels(std::vector<std::string>{"vel_x", "vel_y", "vel_z"});
    log_vel_baselink_.SetLabels(std::vector<std::string>{"baselink_vel_x", "baselink_vel_y", "baselink_vel_z"});
    log_confidence_.SetLabels(std::vector<std::string>{"lidar loc confidence"});
    log_error_.SetLabels(std::vector<std::string>{"err v", "err h", "err eval v", "err eval h"});

    return true;
}

void PangolinWindowImpl::Reset(const std::vector<Keyframe::Ptr> &keyframes) {
    UL lock(mtx_reset_);
    cloud_map_ui_.clear();
    scans_.clear();
    current_scan_ui_ = nullptr;
    traj_scans_->Clear();

    for (const auto &keyframe : keyframes) {
        traj_scans_->AddPt(keyframe->GetOptPose());
    }

    std::size_t i = keyframes.size() > max_size_of_current_scan_ ? keyframes.size() - max_size_of_current_scan_ : 0;
    for (; i < keyframes.size(); ++i) {
        const auto &keyframe = keyframes.at(i);
        current_scan_ui_ = std::make_shared<ui::UiCloud>();
        CloudPtr tmp_cloud = std::make_shared<PointCloudType>(*(keyframe->GetCloud()));
        current_scan_ui_->SetCloud(math::VoxelGrid(tmp_cloud, 0.5), keyframe->GetOptPose());
        current_scan_ui_->SetRenderColor(ui::UiCloud::UseColor::HEIGHT_COLOR);

        scans_.emplace_back(current_scan_ui_);
    }

    newest_backend_pose_ = keyframes.back()->GetOptPose();
}

bool PangolinWindowImpl::DeInit() {
    ReleaseBuffer();
    return true;
}

bool PangolinWindowImpl::UpdateGlobalMap() {
    if (!cloud_global_need_update_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_map_cloud_);
    for (const auto &cp : cloud_global_map_) {
        if (cloud_map_ui_.find(cp.first) != cloud_map_ui_.end()) {
            continue;
        }

        std::shared_ptr<ui::UiCloud> ui_cloud(new ui::UiCloud);
        ui_cloud->SetCloud(cp.second, SE3());
        ui_cloud->SetRenderColor(ui::UiCloud::UseColor::GRAY_COLOR);
        cloud_map_ui_.emplace(cp.first, ui_cloud);
    }

    for (auto iter = cloud_map_ui_.begin(); iter != cloud_map_ui_.end();) {
        if (cloud_global_map_.find(iter->first) == cloud_global_map_.end()) {
            iter = cloud_map_ui_.erase(iter);
        } else {
            iter++;
        }
    }
    cloud_global_need_update_.store(false);

    return true;
}

bool PangolinWindowImpl::UpdateDynamicMap() {
    if (!cloud_dynamic_need_update_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_map_cloud_);
    for (const auto &cp : cloud_dynamic_map_) {
        auto it = cloud_dyn_ui_.find(cp.first);
        if (it != cloud_dyn_ui_.end()) {
            // 存在也要更新
            it->second.reset(new ui::UiCloud);
            // it->second->SetRenderColor(ui::UiCloud::UseColor::PCL_COLOR);
            it->second->SetCustomColor(Vec4f(0.0, 0.2, 1.0, 1.0));
            it->second->SetCloud(cp.second, SE3());
            it->second->SetRenderColor(ui::UiCloud::UseColor::CUSTOM_COLOR);
            continue;
        }

        /// 不存在则创建一个
        std::shared_ptr<ui::UiCloud> ui_cloud(new ui::UiCloud);
        ui_cloud->SetCustomColor(Vec4f(0.0, 0.2, 1.0, 1.0));
        ui_cloud->SetCloud(cp.second, SE3());
        ui_cloud->SetRenderColor(ui::UiCloud::UseColor::CUSTOM_COLOR);
        // ui_cloud->SetRenderColor(ui::UiCloud::UseColor::PCL_COLOR);
        cloud_dyn_ui_.emplace(cp.first, ui_cloud);
    }

    for (auto iter = cloud_dyn_ui_.begin(); iter != cloud_dyn_ui_.end();) {
        if (cloud_dynamic_map_.find(iter->first) == cloud_dynamic_map_.end()) {
            iter = cloud_dyn_ui_.erase(iter);
        } else {
            iter++;
        };
    }

    cloud_dynamic_need_update_.store(false);
    return true;
}

bool PangolinWindowImpl::UpdateCurrentScan() {
    UL lock(mtx_current_scan_);
    if (current_scan_ != nullptr && !current_scan_->empty() && current_scan_need_update_) {
        if (current_scan_ui_) {
            current_scan_ui_->SetRenderColor(ui::UiCloud::UseColor::HEIGHT_COLOR);
            scans_.emplace_back(current_scan_ui_);
        }

        current_scan_ui_ = std::make_shared<ui::UiCloud>();
        current_scan_ui_->SetCloud(current_scan_, current_scan_pose_);
        current_scan_ui_->SetRenderColor(ui::UiCloud::UseColor::HEIGHT_COLOR);

        current_scan_need_update_.store(false);

        traj_scans_->AddPt(current_scan_pose_);

        newest_backend_pose_ = current_scan_pose_;
    }

    while (scans_.size() >= max_size_of_current_scan_) {
        scans_.pop_front();
    }

    return true;
}

bool PangolinWindowImpl::UpdateLoopDebug() {
    std::lock_guard<std::mutex> lock(mtx_loop_info_);
    
    bool has_update = false;
    
    // 检查是否需要更新回环调试点云
    if (loop_debug_need_update_.load()) {
        // 更新回环submap点云 (鲜红色) - 点云已经在map坐标系下
        if (loop_submap_ && !loop_submap_->empty()) {
            loop_submap_ui_ = std::make_shared<ui::UiCloud>();
            loop_submap_ui_->SetCloud(loop_submap_, SE3());  // 点云已在map坐标系
            loop_submap_ui_->SetCustomColor(Vec4f(1.0, 0.0, 0.0, 1.0));  // 鲜红色，完全不透明
            loop_submap_ui_->SetRenderColor(ui::UiCloud::UseColor::CUSTOM_COLOR);
        }
        
        // 更新回环scan点云 (鲜黄色) - 使用当前帧的位姿
        if (loop_scan_ && !loop_scan_->empty()) {
            loop_scan_ui_ = std::make_shared<ui::UiCloud>();
            loop_scan_ui_->SetCloud(loop_scan_, loop_scan_pose_);  // 使用current_kf_pose
            loop_scan_ui_->SetCustomColor(Vec4f(1.0, 1.0, 0.0, 1.0));  // 鲜黄色，完全不透明
            loop_scan_ui_->SetRenderColor(ui::UiCloud::UseColor::CUSTOM_COLOR);
        }
        
        // 更新回环aligned点云 (鲜绿色) - NDT输出的点云已经变换到map坐标系
        if (loop_aligned_ && !loop_aligned_->empty()) {
            loop_aligned_ui_ = std::make_shared<ui::UiCloud>();
            loop_aligned_ui_->SetCloud(loop_aligned_, SE3());  // NDT输出已在map坐标系
            loop_aligned_ui_->SetCustomColor(Vec4f(0.0, 0.0, 1.0, 1.0));  // 蓝色，完全不透明
            loop_aligned_ui_->SetRenderColor(ui::UiCloud::UseColor::CUSTOM_COLOR);
        }
        
        loop_debug_need_update_.store(false);
        has_update = true;
    }
    
    // 累积所有回环边（而不是覆盖），保持历史回环边一直显示
    for (const auto& edge : loop_info_) {
        // 检查是否已存在，避免重复添加
        bool exists = false;
        for (const auto& existing_edge : loop_info_ui_) {
            if (existing_edge.first == edge.first && existing_edge.second == edge.second) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            loop_info_ui_.push_back(edge);
        }
    }
    
    return has_update;
}

bool PangolinWindowImpl::UpdateState() {
    if (!kf_result_need_update_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_nav_state_);
    Vec3d pos = pose_.translation().eval();
    Vec3d vel_baselink = pose_.so3().inverse() * vel_;
    double roll = pose_.angleX();
    double pitch = pose_.angleY();
    double yaw = pose_.angleZ();

    // 滤波器状态作曲线图
    log_vel_.Log(vel_(0), vel_(1), vel_(2));
    log_vel_baselink_.Log(vel_baselink(0), vel_baselink(1), vel_baselink(2));
    log_confidence_.Log(confidence_);

    newest_frontend_pose_ = pose_;
    traj_newest_state_->AddPt(newest_frontend_pose_);

    kf_result_need_update_.store(false);
    return false;
}

void PangolinWindowImpl::DrawAll() {
    /// 地图
    for (const auto &pc : cloud_map_ui_) {
        pc.second->Render();
    }

    /// 动态地图
    for (const auto &pc : cloud_dyn_ui_) {
        pc.second->Render();
    }

    /// 缓存的scans
    for (const auto &s : scans_) {
        s->Render();
    }

    current_scan_ui_->Render();

    if (draw_frontend_traj_) {
        traj_newest_state_->Render();
        // 车
        frontend_car_.SetPose(newest_frontend_pose_);  // 车在current pose上
        frontend_car_.Render();
    }

    if (draw_backend_traj_) {
        traj_scans_->Render();
        // 车
        backend_car_.SetPose(newest_backend_pose_);
        backend_car_.Render();
    }

    // 关键帧
    {
        UL lock(mtx_current_scan_);

        if (all_keyframes_.size() > 1) {
            
            // 绘制关键帧点 (紫色球体)
            glPointSize(8.0);
            glBegin(GL_POINTS);
            glColor3f(0.5, 0.0, 0.5);  // 紫色
            
            for (const auto& kf : all_keyframes_) {
                auto p = kf->GetOptPose().translation();
                glVertex3f(p[0], p[1], p[2]);
            }
            
            glEnd();

            /// 闭环后的轨迹线
            glLineWidth(2.0);
            glBegin(GL_LINE_STRIP);
            glColor3f(0.5, 0.0, 0.5);  // 紫色

            for (int i = 0; i < all_keyframes_.size() - 1; ++i) {
                auto p1 = all_keyframes_[i]->GetOptPose().translation();
                auto p2 = all_keyframes_[i + 1]->GetOptPose().translation();

                glVertex3f(p1[0], p1[1], p1[2]);
                glVertex3f(p2[0], p2[1], p2[2]);
            }

            glEnd();
        }
    }
    
    // 绘制回环边 (绿色线条) - 先复制数据再绘制，避免长时间持锁
    std::deque<std::pair<int, int>> loop_edges_copy;
    {
        std::lock_guard<std::mutex> lock_loop(mtx_loop_info_);
        loop_edges_copy = loop_info_ui_;
    }
    
    if (!loop_edges_copy.empty() && all_keyframes_.size() > 1) {
        UL lock_kf(mtx_current_scan_);
        
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glColor3f(0.0, 1.0, 0.0);  // 绿色
        
        for (const auto& loop_pair : loop_edges_copy) {
            IdType id1 = loop_pair.first;
            IdType id2 = loop_pair.second;
            
            // 检查索引有效性
            if (id1 < all_keyframes_.size() && id2 < all_keyframes_.size()) {
                auto p1 = all_keyframes_[id1]->GetOptPose().translation();
                auto p2 = all_keyframes_[id2]->GetOptPose().translation();
                
                glVertex3f(p1[0], p1[1], p1[2]);
                glVertex3f(p2[0], p2[1], p2[2]);
            }
        }
        
        glEnd();
    }
    
    // 渲染回环调试点云 (最后渲染，显示在最上层，使用更大的点)
    glPointSize(2.5);  // 增大回环点云的点尺寸
    if (loop_submap_ui_) {
        loop_submap_ui_->Render();
    }
    if (loop_scan_ui_) {
        loop_scan_ui_->Render();
    }
    if (loop_aligned_ui_) {
        loop_aligned_ui_->Render();
    }
    glPointSize(1.0);  // 恢复默认点尺寸

    // 绘制 VoxelMap 平面
    if (show_voxel_planes_) {
        DrawVoxelPlanes();
    }

    // 文字
    RenderLabels();
}

bool PangolinWindowImpl::UpdateVoxelPlanes() {
    if (!voxel_planes_need_update_.load()) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mtx_voxel_planes_);
    voxel_planes_need_update_.store(false);
    return true;
}

void PangolinWindowImpl::DrawVoxelPlanes() {
    std::lock_guard<std::mutex> lock(mtx_voxel_planes_);
    if (voxel_planes_.empty()) return;

    // 使用 GL_TRIANGLE_FAN 绘制椭圆平面圆盘，颜色按 plane_cov_trace jet 映射
    constexpr int kSlices = 32;
    constexpr double kTwoPi = 2.0 * M_PI;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for (const auto& p : voxel_planes_) {
        if (!p.is_plane) continue;

        // Jet colormap
        double trace = std::min(p.plane_cov_trace, 0.25) / 0.25;
        trace = std::pow(trace, 0.2);
        float r, g, b;
        if (trace < 0.25) {
            r = 0.0f; g = static_cast<float>(4.0 * trace); b = 1.0f;
        } else if (trace < 0.5) {
            r = 0.0f; g = 1.0f; b = static_cast<float>(1.0 - 4.0 * (trace - 0.25));
        } else if (trace < 0.75) {
            r = static_cast<float>(4.0 * (trace - 0.5)); g = 1.0f; b = 0.0f;
        } else {
            r = 1.0f; g = static_cast<float>(1.0 - 4.0 * (trace - 0.75)); b = 0.0f;
        }
        glColor4f(r, g, b, 0.5f);

        // 椭圆半径（与 RViz MarkerArray 一致）
        double rx = 1.5 * std::sqrt(p.max_eigen_value);
        double ry = 1.5 * std::sqrt(p.mid_eigen_value);

        Eigen::Vector3d cx = p.center;
        Eigen::Vector3d ex = p.x_normal * rx;
        Eigen::Vector3d ey = p.y_normal * ry;

        glBegin(GL_TRIANGLE_FAN);
        glVertex3d(cx(0), cx(1), cx(2));
        for (int s = 0; s <= kSlices; ++s) {
            double theta = kTwoPi * s / kSlices;
            Eigen::Vector3d pt = cx + std::cos(theta) * ex + std::sin(theta) * ey;
            glVertex3d(pt(0), pt(1), pt(2));
        }
        glEnd();
    }

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);  // 恢复默认颜色
    glDisable(GL_BLEND);
}

void PangolinWindowImpl::RenderClouds() {
    UL lock(mtx_reset_);

    // 更新各种推送过来的状态
    UpdateGlobalMap();
    UpdateDynamicMap();
    UpdateState();
    UpdateCurrentScan();
    UpdateLoopDebug();  // 更新回环调试信息
    UpdateVoxelPlanes(); // 更新 VoxelMap 平面

    // 绘制
    pangolin::Display(dis_3d_main_name_).Activate(s_cam_main_);
    DrawAll();
}

void PangolinWindowImpl::RenderLabels() {
    // 定位状态标识，显示在3D窗口中
    auto &d_cam3d_main = pangolin::Display(dis_3d_main_name_);
    d_cam3d_main.Activate(s_cam_main_);
    const auto cur_width = d_cam3d_main.v.w;
    const auto cur_height = d_cam3d_main.v.h;

    GLint view[4];
    glGetIntegerv(GL_VIEWPORT, view);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, cur_width, 0, cur_height, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glTranslatef(5, cur_height - 1.5 * gltext_label_global_.Height(), 1.0);
    glColor3ub(127, 127, 127);
    gltext_label_global_.Draw();

    // Restore modelview / project matrices
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void PangolinWindowImpl::CreateDisplayLayout() {
    // define camera render object (for view / scene browsing)
    // 定义视点的透视投影方式
    auto proj_mat_main = pangolin::ProjectionMatrix(win_width_, win_width_, cam_focus_, cam_focus_, win_width_ / 2,
                                                    win_width_ / 2, cam_z_near_, cam_z_far_);
    // 模型视图矩阵定义了视点的位置和朝向
    auto model_view_main = pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, pangolin::AxisY);
    s_cam_main_ = pangolin::OpenGlRenderState(std::move(proj_mat_main), std::move(model_view_main));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &d_cam3d_main = pangolin::Display(dis_3d_main_name_)
                                       .SetBounds(0.0, 1.0, 0.0, 1.0)
                                       .SetHandler(new pangolin::Handler3D(s_cam_main_));

    pangolin::View &d_cam3d = pangolin::Display(dis_3d_name_)
                                  .SetBounds(0.0, 1.0, 0.0, 0.75)
                                  .SetLayout(pangolin::LayoutOverlay)
                                  .AddDisplay(d_cam3d_main);

    // OpenGL 'view' of data. We might have many views of the same data.
    plotter_vel_ = std::make_unique<pangolin::Plotter>(&log_vel_, -10, 600, -11, 11, 75, 2);
    plotter_vel_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_vel_->Track("$i");
    plotter_vel_baselink_ = std::make_unique<pangolin::Plotter>(&log_vel_baselink_, -10, 600, -11, 11, 75, 2);
    plotter_vel_baselink_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_vel_baselink_->Track("$i");
    plotter_confidence_ = std::make_unique<pangolin::Plotter>(&log_confidence_, -10, 600, 0, 5.0, 100, 0.5);
    plotter_confidence_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_confidence_->Track("$i");
    plotter_err_ = std::make_unique<pangolin::Plotter>(&log_error_, -10, 600, 0, 1.0, 100, 0.1);
    plotter_err_->SetBounds(0.02, 0.98, 0.0, 1.0);
    plotter_err_->Track("$i");

    pangolin::View &d_plot = pangolin::Display(dis_plot_name_)
                                 .SetBounds(0.0, 1.0, 0.75, 1.0)
                                 .SetLayout(pangolin::LayoutEqualVertical)
                                 .AddDisplay(*plotter_confidence_)
                                 .AddDisplay(*plotter_err_)
                                 .AddDisplay(*plotter_vel_)
                                 .AddDisplay(*plotter_vel_baselink_);
    pangolin::Display(dis_main_name_)
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(menu_width_), 1.0)
        .AddDisplay(d_cam3d)
        .AddDisplay(d_plot);
}

void PangolinWindowImpl::Render() {
    pangolin::BindToContext(win_name_);

    // Issue specific OpenGl we might need
    // 启用OpenGL深度测试和混合功能，以支持透明度等效果。
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // menu
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(menu_width_));
    pangolin::Var<bool> menu_follow_loc("menu.Follow", false, true);                     // 跟踪实时定位
    pangolin::Var<bool> menu_show_voxel_planes("menu.Show Voxel Planes", true, true);   // 显示 VoxelMap 平面
    pangolin::Var<bool> menu_draw_frontend_traj("menu.Draw Frontend Traj", true, true);  // 前端实时轨迹
    pangolin::Var<bool> menu_draw_backend_traj("menu.Draw Backend Traj", false, true);   // 后端实时轨迹
    pangolin::Var<bool> menu_reset_3d_view("menu.Reset 3D View", false, false);          // 重置俯视视角
    pangolin::Var<bool> menu_reset_front_view("menu.Set to front View", false, false);   // 前视视角
    pangolin::Var<bool> menu_step("menu.Step", false, false);                            // 单步调试
    pangolin::Var<float> menu_play_speed("menu.Play speed", 10.0, 0.1, 10.0);            // 运行速度
    pangolin::Var<float> menu_intensity("menu.intensity", 0.5, 0.0, 1.0);                // 亮度

    // display layout
    CreateDisplayLayout();

    exit_flag_.store(false);
    while (!pangolin::ShouldQuit() && !exit_flag_) {
        // Clear entire screen
        glClearColor(20.0 / 255.0, 20.0 / 255.0, 20.0 / 255.0, 1.0);
        // 清除了颜色缓冲区（GL_COLOR_BUFFER_BIT）和深度缓冲区（GL_DEPTH_BUFFER_BIT）。
        // 通常在每一帧渲染之前执行的操作，以准备渲染新的内容。
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // menu control
        following_loc_ = menu_follow_loc;
        draw_frontend_traj_ = menu_draw_frontend_traj;
        draw_backend_traj_ = menu_draw_backend_traj;
        show_voxel_planes_ = menu_show_voxel_planes;

        if (menu_reset_3d_view) {
            s_cam_main_.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, 1000, 0, 0, 0, pangolin::AxisY));
            menu_reset_3d_view = false;
        }

        if (menu_reset_front_view) {
            s_cam_main_.SetModelViewMatrix(pangolin::ModelViewLookAt(-50, 0, 10, 50, 0, 10, pangolin::AxisZ));
            menu_reset_front_view = false;
        }

        if (menu_step) {
            debug::flg_next = true;
        } else {
            debug::flg_next = false;
        }

        debug::play_speed = menu_play_speed;
        ui::opacity = menu_intensity;

        // Render pointcloud
        RenderClouds();

        /// 处理相机跟随问题
        if (following_loc_) {
            Eigen::Vector3d translation = newest_frontend_pose_.translation();
            Sophus::SE3d newest_frontend_pose_new(Eigen::Quaterniond::Identity(),
                                                  Eigen::Vector3d(translation.x(), translation.y(), 0.0));
            s_cam_main_.Follow(newest_frontend_pose_new.matrix());
        }

        // Swap frames and Process Events
        // 完成当前帧的渲染并处理与窗口交互相关的事件

        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
    pangolin::DestroyWindow(GetWindowName());
}

std::string PangolinWindowImpl::GetWindowName() const { return win_name_; }

void PangolinWindowImpl::AllocateBuffer() {
    std::string global_text(
        "Welcome to SAD.UI. Open source code: https://github.com/gaoxiang12/slam_in_autonomous_driving. All right "
        "reserved.\n"
        "Red: newest IMU pose, yellow: lidar scan pose");
    auto &font = pangolin::default_font();
    gltext_label_global_ = font.Text(global_text);
}

void PangolinWindowImpl::ReleaseBuffer() {}

}  // namespace lightning::ui
