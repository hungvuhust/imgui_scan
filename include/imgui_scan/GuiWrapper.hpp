#pragma once

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <functional>
#include <string>

class GuiWrapper {
public:
  // Constructor với các tham số cơ bản
  GuiWrapper(int                width  = 1280,
             int                height = 720,
             const std::string& title  = "ImGui Application");

  // Destructor
  ~GuiWrapper();

  // Khởi tạo GUI
  bool Initialize();

  // Chạy main loop
  void Run();

  // Đóng ứng dụng
  void Close();

  // Callback cho việc render GUI
  void SetRenderCallback(std::function<void()> callback);

  // Thiết lập màu nền
  void SetClearColor(float r, float g, float b, float a = 1.0f);

  // Thiết lập VSync
  void SetVSync(bool enabled);

  // Lấy kích thước cửa sổ
  void GetWindowSize(int& width, int& height);

  // Thiết lập kích thước cửa sổ
  void SetWindowSize(int width, int height);

  // Kiểm tra xem cửa sổ có nên đóng không
  bool ShouldClose() const;

  // Lấy GLFWwindow pointer (cho advanced usage)
  GLFWwindow* GetWindow() const {
    return m_window;
  }

  // Lấy ImGuiIO reference
  ImGuiIO& GetIO() {
    return ImGui::GetIO();
  }

  // Lấy FPS hiện tại
  float GetFPS() const {
    return m_fps;
  }

  // Bật/tắt hiển thị FPS overlay
  void SetShowFPS(bool show) {
    m_showFPS = show;
  }
  bool IsShowingFPS() const {
    return m_showFPS;
  }

private:
  GLFWwindow* m_window;
  std::string m_title;
  int         m_width, m_height;
  ImVec4      m_clearColor;
  bool        m_vsyncEnabled;
  bool        m_initialized;

  std::function<void()> m_renderCallback;

  // FPS tracking
  float  m_fps;
  bool   m_showFPS;
  double m_lastTime;
  int    m_frameCount;

  // Callback cho GLFW error
  static void GlfwErrorCallback(int error, const char* description);

  // Cleanup resources
  void Cleanup();

  // Update FPS calculation
  void UpdateFPS();

  // Render FPS overlay
  void RenderFPSOverlay();
};
