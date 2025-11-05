#include "imgui_scan/GuiWrapper.hpp"
#include <iostream>

GuiWrapper::GuiWrapper(int width, int height, const std::string& title)
  : m_window(nullptr),
    m_title(title),
    m_width(width),
    m_height(height),
    m_clearColor(0.45f, 0.55f, 0.60f, 1.00f),
    m_vsyncEnabled(true),
    m_initialized(false),
    m_renderCallback(nullptr),
    m_fps(0.0f),
    m_showFPS(true),
    m_lastTime(0.0),
    m_frameCount(0) {
}

GuiWrapper::~GuiWrapper() {
  Cleanup();
}

void GuiWrapper::GlfwErrorCallback(int error, const char* description) {
  std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

bool GuiWrapper::Initialize() {
  if (m_initialized) {
    return true;
  }

  // Thiết lập GLFW error callback
  glfwSetErrorCallback(GlfwErrorCallback);

  // Khởi tạo GLFW
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW" << std::endl;
    return false;
  }

  // Thiết lập OpenGL version
  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  // Tạo cửa sổ
  m_window =
    glfwCreateWindow(m_width, m_height, m_title.c_str(), nullptr, nullptr);
  if (!m_window) {
    std::cerr << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(m_window);
  glfwSwapInterval(m_vsyncEnabled ? 1 : 0);

  // Khởi tạo ImGui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;

  // Thiết lập style
  ImGui::StyleColorsDark();

  // Khởi tạo ImGui backends
  ImGui_ImplGlfw_InitForOpenGL(m_window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  m_initialized = true;

  // Khởi tạo FPS tracking
  m_lastTime = glfwGetTime();

  return true;
}

void GuiWrapper::Run() {
  if (!m_initialized) {
    std::cerr << "GuiWrapper not initialized. Call Initialize() first."
              << std::endl;
    return;
  }

  while (!glfwWindowShouldClose(m_window)) {
    // Poll events
    glfwPollEvents();

    // Update FPS
    UpdateFPS();

    // Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Gọi render callback nếu có
    if (m_renderCallback) {
      m_renderCallback();
    }

    // Render FPS overlay
    if (m_showFPS) {
      RenderFPSOverlay();
    }

    // Render
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(m_window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(m_clearColor.x,
                 m_clearColor.y,
                 m_clearColor.z,
                 m_clearColor.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(m_window);
  }
}

void GuiWrapper::Close() {
  if (m_window) {
    glfwSetWindowShouldClose(m_window, true);
  }
}

void GuiWrapper::SetRenderCallback(std::function<void()> callback) {
  m_renderCallback = callback;
}

void GuiWrapper::SetClearColor(float r, float g, float b, float a) {
  m_clearColor = ImVec4(r, g, b, a);
}

void GuiWrapper::SetVSync(bool enabled) {
  m_vsyncEnabled = enabled;
  if (m_window) {
    glfwSwapInterval(enabled ? 1 : 0);
  }
}

void GuiWrapper::GetWindowSize(int& width, int& height) {
  if (m_window) {
    glfwGetWindowSize(m_window, &width, &height);
  } else {
    width  = m_width;
    height = m_height;
  }
}

void GuiWrapper::SetWindowSize(int width, int height) {
  m_width  = width;
  m_height = height;
  if (m_window) {
    glfwSetWindowSize(m_window, width, height);
  }
}

bool GuiWrapper::ShouldClose() const {
  return m_window ? glfwWindowShouldClose(m_window) : true;
}

void GuiWrapper::Cleanup() {
  if (m_initialized) {
    // Cleanup ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // Cleanup GLFW
    if (m_window) {
      glfwDestroyWindow(m_window);
      m_window = nullptr;
    }
    glfwTerminate();

    m_initialized = false;
  }
}

void GuiWrapper::UpdateFPS() {
  double currentTime = glfwGetTime();
  m_frameCount++;

  // Cập nhật FPS mỗi giây
  if (currentTime - m_lastTime >= 1.0) {
    m_fps = static_cast<float>(m_frameCount) / (currentTime - m_lastTime);
    m_frameCount = 0;
    m_lastTime   = currentTime;
  }
}

void GuiWrapper::RenderFPSOverlay() {
  // Thiết lập cửa sổ FPS ở góc trái trên
  ImGuiWindowFlags window_flags =
    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
    ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;

  const float          PAD      = 10.0f;
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImVec2 work_pos = viewport->WorkPos;  // Sử dụng work area để tránh menu bar
  ImVec2 window_pos = ImVec2(work_pos.x + PAD, work_pos.y + PAD);

  ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(0.35f);  // Trong suốt

  if (ImGui::Begin("FPS Overlay", nullptr, window_flags)) {
    ImGui::Text("FPS: %.1f", m_fps);
    ImGui::Text("Frame time: %.3f ms", 1000.0f / m_fps);
  }
  ImGui::End();
}
