#include "imgui_scan/GuiWrapper.hpp"
#include "imgui_scan/TFPublish.hpp"
#include "imgui.h"
#include "rclcpp/rclcpp.hpp"
#include <GLFW/glfw3.h>
#include <memory>

// Biến global cho demo
bool        show_demo_window = true;
ImVec4      clear_color      = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
GuiWrapper* g_gui = nullptr;  // Global pointer để access trong callback
std::shared_ptr<TFPublish> g_tf_publisher =
  nullptr;  // Global pointer cho TF publisher

// Biến để lưu giá trị từ slider
static float g_fx   = 0.0f;
static float g_fy   = 0.0f;
static float g_fyaw = 0.0f;
static float g_rx   = 0.0f;
static float g_ry   = 0.0f;
static float g_ryaw = 0.0f;

// Callback function để render GUI
void RenderGUI() {
  // Tạo cửa sổ chính
  ImGui::Begin("imgui_scan");
  ImGui::Text("TF Transform Publisher");

  // Sliders cho x, y, yaw
  ImGui::SliderFloat("Front X", &g_fx, -1.0f, 1.0f);
  ImGui::SliderFloat("Front Y", &g_fy, -1.0f, 1.0f);
  ImGui::SliderFloat("Front Yaw (rad)", &g_fyaw, -3.14159f, 3.14159f);
  ImGui::SliderFloat("Rear X", &g_rx, -1.0f, 1.0f);
  ImGui::SliderFloat("Rear Y", &g_ry, -1.0f, 1.0f);
  ImGui::SliderFloat("Rear Yaw (rad)", &g_ryaw, -3.14159f, 3.14159f);

  // Hiển thị giá trị hiện tại
  ImGui::Text("Current values: Front X=%.3f, Front Y=%.3f, Front Yaw=%.3f",
              g_fx,
              g_fy,
              g_fyaw);
  ImGui::Text("Current values: Rear X=%.3f, Rear Y=%.3f, Rear Yaw=%.3f",
              g_rx,
              g_ry,
              g_ryaw);

  // Publish transforms khi có thay đổi
  if (g_tf_publisher) {
    g_tf_publisher->publishTransforms(static_cast<double>(g_fx),
                                      static_cast<double>(g_fy),
                                      static_cast<double>(g_fyaw),
                                      static_cast<double>(g_rx),
                                      static_cast<double>(g_ry),
                                      static_cast<double>(g_ryaw));
  }

  ImGui::Separator();

  // Checkbox để bật/tắt FPS display
  if (g_gui) {
    bool showFPS = g_gui->IsShowingFPS();
    if (ImGui::Checkbox("Show FPS", &showFPS)) {
      g_gui->SetShowFPS(showFPS);
    }
  }

  ImGui::End();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Tạo TF publisher node
  g_tf_publisher = std::make_shared<TFPublish>();

  // Tạo GUI wrapper
  GuiWrapper gui(480, 360, "ImGui + OpenGL + GLFW Demo với Wrapper");
  g_gui = &gui;  // Lưu pointer global

  // Khởi tạo GUI
  if (!gui.Initialize()) {
    return 1;
  }

  // Thiết lập màu nền ban đầu
  gui.SetClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);

  // Thiết lập render callback
  gui.SetRenderCallback(RenderGUI);

  // Chạy main loop với ROS2 spinner
  // Sử dụng custom loop để tích hợp ROS2 spinner
  GLFWwindow* window      = gui.GetWindow();
  double      last_time   = glfwGetTime();
  int         frame_count = 0;
  float       fps         = 0.0f;

  while (!gui.ShouldClose()) {
    // Poll ROS2 events
    rclcpp::spin_some(g_tf_publisher);

    // Poll GLFW events
    glfwPollEvents();

    // Update FPS (tương tự như GuiWrapper::UpdateFPS)
    double current_time = glfwGetTime();
    frame_count++;
    if (current_time - last_time >= 1.0) {
      fps = static_cast<float>(frame_count) / (current_time - last_time);
      frame_count = 0;
      last_time   = current_time;
    }

    // Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Gọi render callback
    RenderGUI();

    // Render FPS overlay nếu cần
    if (g_gui && g_gui->IsShowingFPS()) {
      ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;

      const float          PAD      = 10.0f;
      const ImGuiViewport* viewport = ImGui::GetMainViewport();
      ImVec2               work_pos = viewport->WorkPos;
      ImVec2 window_pos = ImVec2(work_pos.x + PAD, work_pos.y + PAD);

      ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always);
      ImGui::SetNextWindowBgAlpha(0.35f);

      if (ImGui::Begin("FPS Overlay", nullptr, window_flags)) {
        ImGui::Text("FPS: %.1f", fps);
        ImGui::Text("Frame time: %.3f ms", 1000.0f / fps);
      }
      ImGui::End();
    }

    // Render
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  rclcpp::shutdown();
  return 0;
}
