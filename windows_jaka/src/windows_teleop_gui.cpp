#define UNICODE
#define _UNICODE
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <objidl.h>
#include <commctrl.h>
#include <commdlg.h>
#include <shellapi.h>
#include <gdiplus.h>

#include "control_pipe.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "comdlg32.lib")
#pragma comment(lib, "gdiplus.lib")

namespace {

constexpr COLORREF kBackground = RGB(242, 247, 250);
constexpr COLORREF kSurface = RGB(255, 255, 255);
constexpr COLORREF kSurfaceAlt = RGB(231, 242, 247);
constexpr COLORREF kBorder = RGB(205, 221, 231);
constexpr COLORREF kText = RGB(31, 49, 61);
constexpr COLORREF kMuted = RGB(101, 125, 140);
constexpr COLORREF kAccent = RGB(28, 143, 166);
constexpr COLORREF kAccentDark = RGB(17, 104, 124);
constexpr COLORREF kAccentSoft = RGB(220, 242, 247);
constexpr COLORREF kGreen = RGB(38, 143, 96);
constexpr COLORREF kGreenSoft = RGB(225, 245, 235);
constexpr COLORREF kRed = RGB(202, 72, 72);
constexpr COLORREF kRedSoft = RGB(252, 235, 235);
constexpr COLORREF kDisabledText = RGB(158, 176, 187);
constexpr COLORREF kDisabledFill = RGB(241, 245, 247);
constexpr UINT_PTR kUiTimer = 1;
constexpr UINT_PTR kJogTimer = 2;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr int kMinimumWindowWidth = 1160;
constexpr int kMinimumWindowHeight = 970;
constexpr int kInitialWindowWidth = 1180;
constexpr int kInitialWindowHeight = 990;
constexpr int kHeadingFontHeight = 25;
constexpr int kBodyFontHeight = 19;
constexpr int kLogoMargin = 24;
constexpr int kLogoMaxWidth = 72;
constexpr int kLogoMaxHeight = 64;
constexpr int kLogoTop = 14;

// 数组顺序固定为 J1..J6，单位是角度。
// 跟随臂和操作臂的默认安全姿态是独立配置，不能共用一组值。
constexpr std::array<const wchar_t*, 6> kFollowerDefaultPoseDegrees{
    L"-80", L"45", L"-90", L"0", L"-90", L"-90"};
constexpr std::array<const wchar_t*, 6> kOperatorDefaultPoseDegrees{
    L"100", L"45", L"-90", L"0", L"-90", L"0"};

enum ControlId : int {
    ID_OPERATOR_IP = 100,
    ID_FOLLOWER_IP,
    ID_PORT,
    ID_FILTER,
    ID_CUTOFF,
    ID_ARM_MOTION,
    ID_MODE_TELEOP,
    ID_MODE_JOINT,
    ID_DRY_RUN,
    ID_TARGET_FOLLOWER,
    ID_TARGET_OPERATOR,
    ID_STOP,
    ID_SAFE_POSE,
    ID_EXECUTE_POSE,
    ID_RECORD,
    ID_PLAYBACK,
    ID_LOG,
    ID_STATUS,
    ID_JOYSTICK,
    ID_J1_MINUS,
    ID_J1_PLUS,
    ID_J2_MINUS,
    ID_J2_PLUS,
    ID_J3_MINUS,
    ID_J3_PLUS,
    ID_J4_MINUS,
    ID_J4_PLUS,
    ID_J5_MINUS,
    ID_J5_PLUS,
    ID_J6_MINUS,
    ID_J6_PLUS,
    ID_POSE_J1,
    ID_POSE_J2,
    ID_POSE_J3,
    ID_POSE_J4,
    ID_POSE_J5,
    ID_POSE_J6,
    ID_IDLE_TITLE,
    ID_IDLE_TEXT,
    ID_TELEOP_TITLE,
    ID_TELEOP_TEXT,
    ID_JOINT_TITLE,
    ID_JOINT_TEXT,
};

enum class WorkMode { Idle, Teleop, Joint, DryRun, Record, Playback };
enum class ControlTarget { Follower, Operator };

constexpr wchar_t kFollowerControlPipe[] = L"\\\\.\\pipe\\jaka_dual_teleop";
constexpr wchar_t kOperatorControlPipe[] = L"\\\\.\\pipe\\jaka_operator_teleop";
constexpr std::array<int, 6> kPoseControlIds{
    ID_POSE_J1, ID_POSE_J2, ID_POSE_J3, ID_POSE_J4, ID_POSE_J5, ID_POSE_J6};

struct AppState {
    HWND window{};
    HWND status{};
    HWND log{};
    HWND record{};
    HWND playback{};
    HWND pose_title{};
    HFONT heading_font{};
    HFONT body_font{};
    Gdiplus::Image* logo_image{};
    HBRUSH background_brush{};
    HBRUSH surface_brush{};
    HBRUSH surface_alt_brush{};
    HANDLE child_process{};
    HANDLE child_thread{};
    HANDLE job{};
    HANDLE child_output_read{};
    COLORREF status_color{kMuted};
    bool real_session{false};
    WorkMode mode{WorkMode::Idle};
    ControlTarget joint_target{ControlTarget::Follower};
    std::array<std::array<std::wstring, 6>, 2> safety_pose_text{};
    std::chrono::steady_clock::time_point last_pipe_error{};
    std::filesystem::path record_path;
    std::filesystem::path playback_path;
    int jog_axis{-1};
    double jog_delta{0.0};
    std::filesystem::path root;
    std::vector<HWND> idle_controls;
    std::vector<HWND> teleop_controls;
    std::vector<HWND> joint_controls;
};

AppState g_app;

std::wstring read_text(HWND control) {
    const int length = GetWindowTextLengthW(control);
    if (length <= 0) return {};
    std::vector<wchar_t> buffer(static_cast<std::size_t>(length) + 1, L'\0');
    const int copied = GetWindowTextW(control, buffer.data(), length + 1);
    return std::wstring(buffer.data(), static_cast<std::size_t>(copied));
}

void set_text(HWND control, const wchar_t* value) {
    if (!control) return;
    SetWindowTextW(control, value);
    // Force the old glyphs to be erased before the replacement text is drawn.
    RedrawWindow(control, nullptr, nullptr,
                 RDW_INVALIDATE | RDW_ERASE | RDW_UPDATENOW);
}

std::filesystem::path find_logo_path() {
    const std::array<std::filesystem::path, 2> candidates{
        g_app.root / L"江苏开放大学.png",
        g_app.root.parent_path().parent_path().parent_path() / L"江苏开放大学.png"};
    for (const auto& candidate : candidates) {
        std::error_code error;
        if (std::filesystem::exists(candidate, error) && !error) return candidate;
    }
    return {};
}

void load_logo_image() {
    delete g_app.logo_image;
    g_app.logo_image = nullptr;

    const std::filesystem::path path = find_logo_path();
    if (path.empty()) return;

    Gdiplus::Image* image = Gdiplus::Image::FromFile(path.c_str(), FALSE);
    if (!image || image->GetLastStatus() != Gdiplus::Ok) {
        delete image;
        return;
    }
    g_app.logo_image = image;
}

void draw_logo(HDC dc, int client_width) {
    if (!dc || !g_app.logo_image || client_width <= 0) return;

    const int source_width = static_cast<int>(g_app.logo_image->GetWidth());
    const int source_height = static_cast<int>(g_app.logo_image->GetHeight());
    if (source_width <= 0 || source_height <= 0) return;

    const double scale = std::min(
        static_cast<double>(kLogoMaxWidth) / source_width,
        static_cast<double>(kLogoMaxHeight) / source_height);
    const int logo_width = std::max(1, static_cast<int>(std::lround(source_width * scale)));
    const int logo_height = std::max(1, static_cast<int>(std::lround(source_height * scale)));
    const int logo_x = client_width - kLogoMargin - logo_width;

    Gdiplus::Graphics graphics(dc);
    graphics.SetInterpolationMode(Gdiplus::InterpolationModeHighQualityBicubic);
    graphics.SetPixelOffsetMode(Gdiplus::PixelOffsetModeHighQuality);
    graphics.DrawImage(g_app.logo_image, logo_x, kLogoTop, logo_width, logo_height);
    graphics.Flush(Gdiplus::FlushIntentionSync);
}

void set_status(const std::wstring& value, COLORREF color = kMuted);
void update_mode_ui();
bool process_running();

void append_log(const std::wstring& line) {
    if (g_app.log) {
        const int length = GetWindowTextLengthW(g_app.log);
        SendMessageW(g_app.log, EM_SETSEL, static_cast<WPARAM>(length), static_cast<LPARAM>(length));
        SendMessageW(g_app.log, EM_REPLACESEL, FALSE, reinterpret_cast<LPARAM>((line + L"\r\n").c_str()));
        SendMessageW(g_app.log, EM_SCROLL, SB_BOTTOM, 0);
    }
}

void drain_child_output() {
    if (!g_app.child_output_read) return;
    DWORD available = 0;
    while (PeekNamedPipe(g_app.child_output_read, nullptr, 0, nullptr, &available, nullptr) && available > 0) {
        char buffer[2048]{};
        DWORD read = 0;
        if (!ReadFile(g_app.child_output_read, buffer, sizeof(buffer) - 1, &read, nullptr) || read == 0) break;
        buffer[read] = '\0';
        std::string chunk(buffer, read);
        std::size_t start = 0;
        while (start < chunk.size()) {
            const std::size_t end = chunk.find_first_of("\r\n", start);
            const std::string line = chunk.substr(start, end == std::string::npos ? std::string::npos : end - start);
            if (!line.empty()) {
                const std::wstring wide_line(line.begin(), line.end());
                append_log(wide_line);
                if (line.find("REAL ROBOT MOTION ENABLED") != std::string::npos) {
                    set_status(g_app.mode == WorkMode::Joint ? L"关节控制运行中" :
                               (g_app.mode == WorkMode::Record ? L"轨迹录制运行中" :
                                (g_app.mode == WorkMode::Playback ? L"轨迹回放运行中" : L"遥操作运行中")), kAccentDark);
                } else if (line.find("DRY-RUN ONLY") != std::string::npos) {
                    set_status(L"Dry-run 检查中", kGreen);
                } else if (line.find("follower login_ret=0") != std::string::npos && g_app.mode == WorkMode::Idle) {
                    set_status(L"跟随臂已连接", kGreen);
                } else if (line.find("operator login_ret=0") != std::string::npos && g_app.mode == WorkMode::Idle) {
                    set_status(L"操作臂已连接", kGreen);
                } else if (line.find("stop_reason=") != std::string::npos) {
                    set_status(L"会话已结束", kMuted);
                }
            }
            if (end == std::string::npos) break;
            start = end + 1;
            while (start < chunk.size() && (chunk[start] == '\r' || chunk[start] == '\n')) ++start;
        }
    }
}

const wchar_t* control_target_label(ControlTarget target) {
    return target == ControlTarget::Operator ? L"操作臂" : L"跟随臂";
}

const wchar_t* control_pipe_for_target(ControlTarget target) {
    return target == ControlTarget::Operator ? kOperatorControlPipe : kFollowerControlPipe;
}

const std::array<int, 6>& pose_control_ids() {
    return kPoseControlIds;
}

std::size_t pose_target_index(ControlTarget target) {
    return target == ControlTarget::Operator ? 1U : 0U;
}

const std::array<const wchar_t*, 6>& default_pose_for_target(ControlTarget target) {
    return target == ControlTarget::Operator ? kOperatorDefaultPoseDegrees
                                             : kFollowerDefaultPoseDegrees;
}

void initialize_safety_pose_text() {
    for (std::size_t i = 0; i < 6; ++i) {
        g_app.safety_pose_text[0][i] = kFollowerDefaultPoseDegrees[i];
        g_app.safety_pose_text[1][i] = kOperatorDefaultPoseDegrees[i];
    }
}

void capture_safety_pose_text(ControlTarget target) {
    auto& values = g_app.safety_pose_text[pose_target_index(target)];
    const auto& ids = pose_control_ids();
    for (std::size_t i = 0; i < ids.size(); ++i) {
        values[i] = read_text(GetDlgItem(g_app.window, ids[i]));
    }
}

void load_safety_pose_text(ControlTarget target) {
    const auto& defaults = default_pose_for_target(target);
    auto& values = g_app.safety_pose_text[pose_target_index(target)];
    const auto& ids = pose_control_ids();
    for (std::size_t i = 0; i < ids.size(); ++i) {
        // A target can be selected before its cache has ever been captured.
        if (values[i].empty()) values[i] = defaults[i];
        set_text(GetDlgItem(g_app.window, ids[i]), values[i].c_str());
    }
}

void update_pose_heading() {
    if (!g_app.pose_title) return;
    std::wstring heading = L"安全姿态参数（当前对象：";
    heading += control_target_label(g_app.joint_target);
    heading += L"，单位：角度 °）";
    set_text(g_app.pose_title, heading.c_str());
}

bool send_pipe_line(const std::string& line) {
    const ControlTarget target = g_app.joint_target;
    if (!windows_jaka::send_control_command(control_pipe_for_target(target), line)) {
        const auto now = std::chrono::steady_clock::now();
        if (g_app.last_pipe_error.time_since_epoch().count() == 0 ||
            now - g_app.last_pipe_error > std::chrono::seconds(1)) {
            g_app.last_pipe_error = now;
            append_log(std::wstring(L"无法连接 ") + control_target_label(target) +
                       L" 命令通道（请确认对应进程已启动）");
        }
        return false;
    }
    g_app.last_pipe_error = {};
    return true;
}

void set_jog_buttons(bool enabled) {
    for (int id = ID_J1_MINUS; id <= ID_J6_PLUS; ++id) {
        EnableWindow(GetDlgItem(g_app.window, id), enabled ? TRUE : FALSE);
    }
}

LRESULT CALLBACK jog_button_subclass(HWND hwnd, UINT message, WPARAM wparam,
                                     LPARAM lparam, UINT_PTR, DWORD_PTR data) {
    const int axis = static_cast<int>(data) / 2;
    const bool positive = (static_cast<int>(data) % 2) != 0;
    const double delta = (axis < 3 ? 0.002 : 0.0015) * (positive ? 1.0 : -1.0);
    if (message == WM_LBUTTONDOWN) {
        g_app.jog_axis = axis;
        g_app.jog_delta = delta;
        SetCapture(hwnd);
        send_pipe_line("JOG " + std::to_string(axis) + " " + std::to_string(delta));
        return 0;
    }
    if (message == WM_LBUTTONUP || message == WM_CAPTURECHANGED) {
        if (g_app.jog_axis == axis) {
            send_pipe_line("JOG_STOP " + std::to_string(axis));
            g_app.jog_axis = -1;
            g_app.jog_delta = 0.0;
        }
        if (GetCapture() == hwnd) ReleaseCapture();
        return 0;
    }
    return DefSubclassProc(hwnd, message, wparam, lparam);
}

bool read_pose_degrees(windows_jaka::JointArray& pose_degrees) {
    const auto& ids = pose_control_ids();
    try {
        for (int i = 0; i < 6; ++i) {
            pose_degrees[i] = std::stod(read_text(GetDlgItem(g_app.window, ids[static_cast<std::size_t>(i)])));
        }
    } catch (...) {
        return false;
    }
    for (double value : pose_degrees) {
        if (!std::isfinite(value) || value < -180.0 || value > 180.0) return false;
    }
    return true;
}

void preview_pose() {
    windows_jaka::JointArray pose_degrees{};
    if (!read_pose_degrees(pose_degrees)) {
        MessageBoxW(g_app.window, L"请输入 6 个有效的关节角（角度，范围 -180° 到 180°）。",
                    L"安全姿态参数无效", MB_ICONWARNING | MB_OK);
        return;
    }
    std::wstringstream output;
    output << std::fixed << std::setprecision(3);
    output << control_target_label(g_app.joint_target) << L" 安全姿态预览(°): [";
    for (int i = 0; i < 6; ++i) {
        if (i) output << L", ";
        output << pose_degrees[i];
    }
    output << L"]";
    append_log(output.str());
}

void execute_pose() {
    if (SendMessageW(GetDlgItem(g_app.window, ID_ARM_MOTION), BM_GETCHECK, 0, 0) != BST_CHECKED) {
        MessageBoxW(g_app.window, L"执行安全姿态前必须勾选“真实运动授权”。",
                    L"真实运动已锁定", MB_ICONWARNING | MB_OK);
        return;
    }
    if (!process_running() || !g_app.real_session || g_app.mode != WorkMode::Joint) {
        MessageBoxW(g_app.window, L"请先进入“关节控制”模式，再执行安全姿态。",
                    L"控制模式不匹配", MB_ICONWARNING | MB_OK);
        return;
    }
    windows_jaka::JointArray pose_degrees{};
    if (!read_pose_degrees(pose_degrees)) {
        MessageBoxW(g_app.window, L"请输入 6 个有效的关节角（角度，范围 -180° 到 180°）。",
                    L"安全姿态参数无效", MB_ICONWARNING | MB_OK);
        return;
    }

    windows_jaka::JointArray pose_radians{};
    for (int i = 0; i < 6; ++i) {
        pose_radians[i] = pose_degrees[i] * kDegreesToRadians;
    }

    std::ostringstream output;
    output << std::fixed << std::setprecision(9) << "SAFEPOSE";
    for (double value : pose_radians) output << ' ' << value;

    std::wstringstream confirmation;
    confirmation << std::fixed << std::setprecision(3);
    confirmation << L"将以低速执行以下" << control_target_label(g_app.joint_target)
                 << L"关节姿态（角度°）：\n[";
    for (int i = 0; i < 6; ++i) {
        if (i) confirmation << L", ";
        confirmation << pose_degrees[i];
    }
    confirmation << L"]\n\n请确认路径无障碍，并确保现场人员可立即按下急停。";
    if (MessageBoxW(g_app.window, confirmation.str().c_str(), L"确认执行安全姿态",
                    MB_ICONWARNING | MB_YESNO | MB_DEFBUTTON2) != IDYES) return;
    if (send_pipe_line(output.str())) {
        append_log(std::wstring(L"已发送安全姿态命令，等待 ") +
                   control_target_label(g_app.joint_target) + L" 执行");
    }
}

HWND make_control(const wchar_t* klass, const wchar_t* text, DWORD style,
                  int x, int y, int width, int height, int id) {
    HWND control = CreateWindowExW(0, klass, text, style | WS_CLIPSIBLINGS,
                                   x, y, width, height, g_app.window,
                                   reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
                                   GetModuleHandleW(nullptr), nullptr);
    if (control) SendMessageW(control, WM_SETFONT,
                              reinterpret_cast<WPARAM>(g_app.body_font), TRUE);
    return control;
}

void set_status(const std::wstring& value, COLORREF color) {
    g_app.status_color = color;
    if (!g_app.status) return;
    set_text(g_app.status, value.c_str());
    InvalidateRect(g_app.status, nullptr, TRUE);
}

std::wstring quote(const std::wstring& value) {
    return L"\"" + value + L"\"";
}

bool process_running() {
    if (!g_app.child_process) return false;
    const DWORD result = WaitForSingleObject(g_app.child_process, 0);
    return result == WAIT_TIMEOUT;
}

bool motion_environment_enabled() {
    wchar_t value[8]{};
    const DWORD length = GetEnvironmentVariableW(L"JAKA_ENABLE_MOTION", value, 8);
    return length == 1 && value[0] == L'1';
}

bool is_real_mode(WorkMode mode) {
    return mode == WorkMode::Teleop || mode == WorkMode::Joint ||
           mode == WorkMode::Record || mode == WorkMode::Playback;
}

const wchar_t* mode_label(WorkMode mode) {
    switch (mode) {
    case WorkMode::Teleop: return L"遥操作";
    case WorkMode::Joint: return L"关节控制";
    case WorkMode::DryRun: return L"Dry-run";
    case WorkMode::Record: return L"轨迹录制";
    case WorkMode::Playback: return L"轨迹回放";
    default: return L"待机";
    }
}

bool set_controls_visible(const std::vector<HWND>& controls, bool visible) {
    bool changed = false;
    for (HWND control : controls) {
        if (!control) continue;
        const bool currently_visible = IsWindowVisible(control) != FALSE;
        if (currently_visible != visible) {
            ShowWindow(control, visible ? SW_SHOW : SW_HIDE);
            changed = true;
        }
    }
    return changed;
}

void update_mode_ui() {
    const bool running = process_running();
    const bool armed = SendMessageW(GetDlgItem(g_app.window, ID_ARM_MOTION),
                                    BM_GETCHECK, 0, 0) == BST_CHECKED;
    const bool record_active = running && g_app.mode == WorkMode::Record;
    const bool joint_active = running && g_app.real_session &&
        (g_app.mode == WorkMode::Joint || g_app.mode == WorkMode::Record);

    EnableWindow(GetDlgItem(g_app.window, ID_MODE_TELEOP), (!running && armed) ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_app.window, ID_MODE_JOINT), (!running && armed) ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_app.window, ID_DRY_RUN), running ? FALSE : TRUE);
    EnableWindow(GetDlgItem(g_app.window, ID_ARM_MOTION), running ? FALSE : TRUE);
    EnableWindow(GetDlgItem(g_app.window, ID_TARGET_FOLLOWER), running ? FALSE : TRUE);
    EnableWindow(GetDlgItem(g_app.window, ID_TARGET_OPERATOR), running ? FALSE : TRUE);
    EnableWindow(GetDlgItem(g_app.window, ID_STOP), running ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_app.window, ID_RECORD),
                 record_active || (!running && armed) ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_app.window, ID_PLAYBACK),
                 (!running && armed) ? TRUE : FALSE);
    set_jog_buttons(joint_active);
    EnableWindow(GetDlgItem(g_app.window, ID_EXECUTE_POSE),
                 (joint_active && g_app.mode == WorkMode::Joint) ? TRUE : FALSE);

    set_text(g_app.record, record_active ? L"停止录制" : L"轨迹录制");
    set_text(g_app.playback, L"轨迹回放");

    bool page_changed = false;
    const bool idle_page = g_app.mode == WorkMode::Idle || g_app.mode == WorkMode::DryRun ||
                           g_app.mode == WorkMode::Playback;
    if (set_controls_visible(g_app.idle_controls, idle_page)) page_changed = true;
    if (set_controls_visible(g_app.teleop_controls, g_app.mode == WorkMode::Teleop)) page_changed = true;
    if (set_controls_visible(g_app.joint_controls,
                             g_app.mode == WorkMode::Joint || g_app.mode == WorkMode::Record)) {
        page_changed = true;
    }

    if (g_app.mode == WorkMode::DryRun) {
        set_text(GetDlgItem(g_app.window, ID_IDLE_TITLE), L"Dry-run 检查中");
        set_text(GetDlgItem(g_app.window, ID_IDLE_TEXT),
                 L"仅验证登录、UDP、状态读取和映射逻辑。\r\n不会调用 servo_j，也不会产生跟随动作。\r\n关节 +/-、轨迹录制与回放保持禁用。");
    } else if (g_app.mode == WorkMode::Playback) {
        std::wstring title = L"轨迹回放中（";
        title += control_target_label(g_app.joint_target);
        title += L"）";
        std::wstring body = L"轨迹按相对起点回放，并受现有滤波、速度、加速度和步长限制。\r\n";
        if (!g_app.playback_path.empty()) body += L"文件：" + g_app.playback_path.wstring() + L"\r\n";
        body += L"需要立即停止时点击“停止”。";
        set_text(GetDlgItem(g_app.window, ID_IDLE_TITLE), title.c_str());
        set_text(GetDlgItem(g_app.window, ID_IDLE_TEXT), body.c_str());
    } else {
        set_text(GetDlgItem(g_app.window, ID_IDLE_TITLE), L"等待选择控制模式");
        set_text(GetDlgItem(g_app.window, ID_IDLE_TEXT),
                 L"1. 勾选真实运动授权并确认现场安全。\r\n2. 选择“遥操作”“关节控制”“轨迹录制”或“轨迹回放”。\r\n3. 切换模式前请先点击“停止”。");
    }
    RedrawWindow(GetDlgItem(g_app.window, ID_IDLE_TITLE), nullptr, nullptr,
                 RDW_INVALIDATE | RDW_ERASE | RDW_UPDATENOW);
    RedrawWindow(GetDlgItem(g_app.window, ID_IDLE_TEXT), nullptr, nullptr,
                 RDW_INVALIDATE | RDW_ERASE | RDW_UPDATENOW);

    std::wstring joint_title;
    std::wstring joint_text;
    if (g_app.mode == WorkMode::Record) {
        joint_title = L"轨迹录制中（";
        joint_title += control_target_label(g_app.joint_target);
        joint_title += L"）";
        joint_text = L"正在记录该机械臂的实际关节角和时间戳。可物理缓慢拖动，或按住 J1-J6 的 +/- 点动。\r\n";
        if (!g_app.record_path.empty()) joint_text += L"文件：" + g_app.record_path.wstring() + L"\r\n";
        joint_text += L"完成后点击“停止录制”或“停止”。";
    } else {
        joint_title = L"关节控制模式已启用（";
        joint_title += control_target_label(g_app.joint_target);
        joint_title += L"）";
        joint_text = L"按住 +/- 点动 ";
        joint_text += control_target_label(g_app.joint_target);
        joint_text += L" 的对应关节，松开后立即停止；安全姿态也作用于同一控制对象。";
        if (g_app.joint_target == ControlTarget::Operator) {
            joint_text += L"\r\n操作臂必须退出拖拽模式，遥操作与关节控制互斥。";
        } else {
            joint_text += L"\r\n此模式忽略操作臂拖拽跟随，避免两种控制方式冲突。";
        }
    }
    set_text(GetDlgItem(g_app.window, ID_JOINT_TITLE), joint_title.c_str());
    set_text(GetDlgItem(g_app.window, ID_JOINT_TEXT), joint_text.c_str());
    update_pose_heading();

    for (int id : {ID_MODE_TELEOP, ID_MODE_JOINT, ID_DRY_RUN, ID_RECORD, ID_PLAYBACK,
                   ID_TARGET_FOLLOWER, ID_TARGET_OPERATOR}) {
        InvalidateRect(GetDlgItem(g_app.window, id), nullptr, TRUE);
    }
    if (page_changed) {
        RedrawWindow(g_app.window, nullptr, nullptr,
                     RDW_INVALIDATE | RDW_ERASE | RDW_ALLCHILDREN | RDW_UPDATENOW);
    }
}

void close_child_handles() {
    if (g_app.child_thread) { CloseHandle(g_app.child_thread); g_app.child_thread = nullptr; }
    if (g_app.child_process) { CloseHandle(g_app.child_process); g_app.child_process = nullptr; }
    if (g_app.job) { CloseHandle(g_app.job); g_app.job = nullptr; }
    if (g_app.child_output_read) { CloseHandle(g_app.child_output_read); g_app.child_output_read = nullptr; }
}

void stop_child() {
    if (!g_app.child_process) {
        g_app.mode = WorkMode::Idle;
        g_app.real_session = false;
        update_mode_ui();
        set_status(L"待机", kMuted);
        return;
    }
    if (process_running()) {
        // Ask both arm processes to stop first so their SDK cleanup and the
        // PowerShell wrapper's child cleanup both run. If the wrapper does not
        // exit, the job object below terminates the whole process tree.
        windows_jaka::send_control_command(kFollowerControlPipe, "STOP");
        windows_jaka::send_control_command(kOperatorControlPipe, "STOP");
        if (WaitForSingleObject(g_app.child_process, 3000) == WAIT_TIMEOUT) {
            if (g_app.job) TerminateJobObject(g_app.job, 1);
            else TerminateProcess(g_app.child_process, 1);
            WaitForSingleObject(g_app.child_process, 1000);
        }
    }
    close_child_handles();
    g_app.real_session = false;
    g_app.mode = WorkMode::Idle;
    g_app.jog_axis = -1;
    g_app.jog_delta = 0.0;
    update_mode_ui();
    set_status(L"待机", kMuted);
    append_log(L"当前控制模式已停止，可重新选择“遥操作”“关节控制”“轨迹录制”或“轨迹回放”");
}

bool start_child(WorkMode mode) {
    if (!is_real_mode(mode) && mode != WorkMode::DryRun) return false;

    const bool real_motion = is_real_mode(mode);
    if (real_motion &&
        SendMessageW(GetDlgItem(g_app.window, ID_ARM_MOTION), BM_GETCHECK, 0, 0) != BST_CHECKED) {
        MessageBoxW(g_app.window, L"请先勾选“真实运动授权”，并确认现场安全。",
                    L"真实运动已锁定", MB_ICONWARNING | MB_OK);
        return false;
    }
    if (real_motion && !motion_environment_enabled()) {
        const std::wstring message =
            L"当前 GUI 进程没有获得真实运动授权。\n\n"
            L"请关闭本窗口，双击便携包根目录中的“启动控制台.cmd”。\n"
            L"该入口会自动为本次 GUI 进程设置 JAKA_ENABLE_MOTION=1，无需手工输入 PowerShell 命令。\n\n"
            L"GUI 仍默认使用 Dry-run；进入界面后还必须勾选“真实运动授权（现场确认安全）”。";
        set_status(L"真实运动环境未授权", kRed);
        append_log(L"当前进程未通过启动控制台获得 JAKA_ENABLE_MOTION=1；已拒绝启动真实运动");
        MessageBoxW(g_app.window, message.c_str(), L"真实运动环境未授权",
                    MB_ICONWARNING | MB_OK);
        return false;
    }

    if (process_running()) {
        set_status(L"会话已在运行，请先停止", kRed);
        append_log(std::wstring(L"当前正在运行“") + mode_label(g_app.mode) + L"”；请先点击“停止”再切换模式");
        MessageBoxW(g_app.window,
                    L"当前已有控制会话正在运行。\n\n请先点击“停止”，等待会话退出后再选择其他模式。",
                    L"请先停止当前会话", MB_ICONINFORMATION | MB_OK);
        return false;
    }

    const std::wstring operator_ip = read_text(GetDlgItem(g_app.window, ID_OPERATOR_IP));
    const std::wstring follower_ip = read_text(GetDlgItem(g_app.window, ID_FOLLOWER_IP));
    const std::wstring port = read_text(GetDlgItem(g_app.window, ID_PORT));
    const std::wstring cutoff = read_text(GetDlgItem(g_app.window, ID_CUTOFF));
    const int filter_index = static_cast<int>(SendMessageW(
        GetDlgItem(g_app.window, ID_FILTER), CB_GETCURSEL, 0, 0));
    const wchar_t* filter = filter_index == 1 ? L"lpf" : (filter_index == 2 ? L"nlf" : L"none");

    std::filesystem::path script = g_app.root / L"run_windows_teleop.ps1";
    if (!std::filesystem::exists(script)) {
        const auto workspace_root = g_app.root.parent_path().parent_path().parent_path();
        script = workspace_root / L"windows_jaka" / L"run_windows_teleop.ps1";
    }
    if (!std::filesystem::exists(script)) {
        append_log(L"未找到 run_windows_teleop.ps1，请检查程序目录或项目目录");
        return false;
    }

    g_app.joint_target =
        SendMessageW(GetDlgItem(g_app.window, ID_TARGET_OPERATOR), BM_GETCHECK, 0, 0) == BST_CHECKED
            ? ControlTarget::Operator : ControlTarget::Follower;

    const wchar_t* follower_control_mode = L"idle";
    const wchar_t* operator_control_mode = L"idle";
    if (mode == WorkMode::Teleop) {
        follower_control_mode = L"teleop";
        operator_control_mode = L"teleop";
    } else if (mode == WorkMode::Joint) {
        if (g_app.joint_target == ControlTarget::Operator) {
            operator_control_mode = L"joint";
        } else {
            follower_control_mode = L"joint";
        }
    } else if (mode == WorkMode::Record || mode == WorkMode::Playback) {
        const wchar_t* selected_mode = mode == WorkMode::Record ? L"record" : L"playback";
        if (g_app.joint_target == ControlTarget::Operator) {
            operator_control_mode = selected_mode;
        } else {
            follower_control_mode = selected_mode;
        }
    }

    std::wstring command = L"powershell.exe -NoProfile -ExecutionPolicy Bypass -File " + quote(script.wstring())
        + L" -OperatorIp " + quote(operator_ip)
        + L" -FollowerIp " + quote(follower_ip)
        + L" -Port " + quote(port)
        + L" -Filter " + filter
        + L" -LpfCutoff " + quote(cutoff)
        + L" -MaxVelocity 1.0 -MaxAcceleration 8.0"
        + L" -ControlPipe " + quote(kFollowerControlPipe)
        + L" -ControlMode " + follower_control_mode
        + L" -OperatorControlPipe " + quote(kOperatorControlPipe)
        + L" -OperatorControlMode " + operator_control_mode;
    if (real_motion) command += L" -ArmMotion";
    else command += L" -DryRun";
    if (mode == WorkMode::Record) {
        const std::wstring option = g_app.joint_target == ControlTarget::Operator
            ? L" -RecordFile " : L" -FollowerRecordFile ";
        command += option + quote(g_app.record_path.wstring());
    } else if (mode == WorkMode::Playback) {
        const std::wstring option = g_app.joint_target == ControlTarget::Operator
            ? L" -PlaybackFile " : L" -FollowerPlaybackFile ";
        command += option + quote(g_app.playback_path.wstring());
    }

    SECURITY_ATTRIBUTES security{};
    security.nLength = sizeof(security);
    security.bInheritHandle = TRUE;
    HANDLE output_read = nullptr;
    HANDLE output_write = nullptr;
    if (!CreatePipe(&output_read, &output_write, &security, 0)) return false;
    SetHandleInformation(output_read, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdOutput = output_write;
    startup.hStdError = output_write;
    PROCESS_INFORMATION process{};
    std::vector<wchar_t> command_line(command.begin(), command.end());
    command_line.push_back(L'\0');
    const BOOL created = CreateProcessW(nullptr, command_line.data(), nullptr, nullptr, TRUE,
                                        CREATE_NO_WINDOW | CREATE_NEW_PROCESS_GROUP,
                                        nullptr, g_app.root.c_str(), &startup, &process);
    CloseHandle(output_write);
    if (!created) {
        CloseHandle(output_read);
        append_log(L"启动 PowerShell 会话失败");
        return false;
    }

    g_app.child_process = process.hProcess;
    g_app.child_thread = process.hThread;
    g_app.child_output_read = output_read;
    g_app.job = CreateJobObjectW(nullptr, nullptr);
    if (g_app.job) {
        JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
        limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
        if (!SetInformationJobObject(g_app.job, JobObjectExtendedLimitInformation,
                                     &limits, sizeof(limits)) ||
            !AssignProcessToJobObject(g_app.job, g_app.child_process)) {
            CloseHandle(g_app.job);
            g_app.job = nullptr;
        }
    }

    g_app.real_session = real_motion;
    g_app.mode = mode;
    update_mode_ui();

    if (mode == WorkMode::Teleop) {
        set_status(L"遥操作 / 正在启动", kAccentDark);
        append_log(L"已请求启动遥操作模式；请等待操作臂和跟随臂连接完成");
    } else if (mode == WorkMode::Joint) {
        const std::wstring target = control_target_label(g_app.joint_target);
        set_status(target + L"关节控制 / 正在启动", kAccentDark);
        append_log(L"已请求启动" + target + L"关节控制模式；连接完成后按住 +/- 点动关节");
    } else if (mode == WorkMode::Record) {
        const std::wstring target = control_target_label(g_app.joint_target);
        set_status(target + L"轨迹录制 / 正在启动", kAccentDark);
        append_log(L"已请求启动" + target + L"轨迹录制；文件：" + g_app.record_path.wstring());
    } else if (mode == WorkMode::Playback) {
        const std::wstring target = control_target_label(g_app.joint_target);
        set_status(target + L"轨迹回放 / 正在启动", kAccentDark);
        append_log(L"已请求启动" + target + L"轨迹回放；文件：" + g_app.playback_path.wstring());
    } else {
        set_status(L"Dry-run / 正在启动", kGreen);
        append_log(L"Dry-run 检查会话已启动；该模式不会调用 servo_j");
    }
    append_log(L"滤波器=" + std::wstring(filter) + L"；LPF 截止参数=" + cutoff);
    return true;
}

std::filesystem::path make_trajectory_path() {
    const auto log_dir = g_app.root / L"logs";
    std::error_code error;
    std::filesystem::create_directories(log_dir, error);
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm local{};
    localtime_s(&local, &now);
    std::wstringstream filename;
    const wchar_t* target = g_app.joint_target == ControlTarget::Operator
        ? L"operator" : L"follower";
    filename << L"trajectory_" << target << L'_' << std::put_time(&local, L"%Y%m%d_%H%M%S") << L".csv";
    return log_dir / filename.str();
}

void toggle_recording() {
    if (process_running()) {
        if (g_app.mode != WorkMode::Record) {
            MessageBoxW(g_app.window, L"当前已有控制会话正在运行。\n\n请先点击“停止”，再开始轨迹录制。",
                        L"请先停止当前会话", MB_ICONINFORMATION | MB_OK);
            return;
        }
        stop_child();
        append_log(L"轨迹录制已停止");
        return;
    }

    g_app.record_path = make_trajectory_path();
    if (g_app.record_path.empty()) {
        append_log(L"无法创建轨迹录制目录");
        return;
    }
    if (start_child(WorkMode::Record)) {
        append_log(L"轨迹录制已开始，目标：" + std::wstring(control_target_label(g_app.joint_target)));
    }
}

void choose_and_playback() {
    if (process_running()) {
        MessageBoxW(g_app.window, L"当前已有控制会话正在运行。\n\n请先点击“停止”，再选择轨迹回放。",
                    L"请先停止当前会话", MB_ICONINFORMATION | MB_OK);
        return;
    }

    wchar_t file_name[MAX_PATH]{};
    const auto log_dir = g_app.root / L"logs";
    const std::wstring initial_dir = std::filesystem::exists(log_dir) ? log_dir.wstring() : g_app.root.wstring();
    OPENFILENAMEW dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.hwndOwner = g_app.window;
    dialog.lpstrFilter = L"JAKA 关节轨迹 (*.csv)\0*.csv\0所有文件 (*.*)\0*.*\0\0";
    dialog.lpstrFile = file_name;
    dialog.nMaxFile = static_cast<DWORD>(std::size(file_name));
    dialog.lpstrInitialDir = initial_dir.c_str();
    dialog.lpstrTitle = L"选择关节轨迹文件";
    dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_EXPLORER | OFN_NOCHANGEDIR;
    if (!GetOpenFileNameW(&dialog)) return;

    g_app.playback_path = file_name;
    const std::wstring target = control_target_label(g_app.joint_target);
    const std::wstring confirm =
        L"即将在" + target + L"上回放所选关节轨迹。\n\n"
        L"请确认目标臂已使能、运动范围无人无障碍、急停可用，并已先进行 Dry-run 或低速验证。\n\n"
        L"回放过程中可随时点击“停止”终止。是否继续？";
    if (MessageBoxW(g_app.window, confirm.c_str(), L"确认关节轨迹回放",
                    MB_YESNO | MB_ICONWARNING | MB_DEFBUTTON2) != IDYES) {
        return;
    }
    if (start_child(WorkMode::Playback)) {
        append_log(L"轨迹回放已开始，目标：" + std::wstring(control_target_label(g_app.joint_target)));
    }
}

COLORREF mix_color(COLORREF a, COLORREF b, int b_percent) {
    const int a_percent = 100 - b_percent;
    return RGB(
        (GetRValue(a) * a_percent + GetRValue(b) * b_percent) / 100,
        (GetGValue(a) * a_percent + GetGValue(b) * b_percent) / 100,
        (GetBValue(a) * a_percent + GetBValue(b) * b_percent) / 100);
}

void draw_section(HDC dc, int x, int y, int width, int height, COLORREF fill = kSurface) {
    HBRUSH brush = CreateSolidBrush(fill);
    HPEN pen = CreatePen(PS_SOLID, 1, kBorder);
    HGDIOBJ old_brush = SelectObject(dc, brush);
    HGDIOBJ old_pen = SelectObject(dc, pen);
    RoundRect(dc, x, y, x + width, y + height, 16, 16);
    SelectObject(dc, old_pen);
    SelectObject(dc, old_brush);
    DeleteObject(pen);
    DeleteObject(brush);
}

void draw_owner_button(const DRAWITEMSTRUCT& item) {
    if (item.CtlType != ODT_BUTTON) return;

    const int id = static_cast<int>(item.CtlID);
    const bool disabled = (item.itemState & ODS_DISABLED) != 0;
    const bool pressed = (item.itemState & ODS_SELECTED) != 0;
    const bool mode_button = id == ID_MODE_TELEOP || id == ID_MODE_JOINT;
    const bool jog_button = id >= ID_J1_MINUS && id <= ID_J6_PLUS;
    const bool active_mode = (id == ID_MODE_TELEOP && g_app.mode == WorkMode::Teleop) ||
                             (id == ID_MODE_JOINT && g_app.mode == WorkMode::Joint) ||
                             (id == ID_DRY_RUN && g_app.mode == WorkMode::DryRun) ||
                             (id == ID_RECORD && g_app.mode == WorkMode::Record) ||
                             (id == ID_PLAYBACK && g_app.mode == WorkMode::Playback);

    COLORREF fill = kSurfaceAlt;
    COLORREF border = kBorder;
    COLORREF text = kText;

    if (active_mode) {
        fill = kAccent;
        border = kAccent;
        text = kSurface;
    } else if (disabled) {
        fill = kDisabledFill;
        border = kBorder;
        text = kDisabledText;
    } else if (mode_button) {
        fill = kAccentSoft;
        border = kAccent;
        text = kAccentDark;
    } else if (id == ID_STOP || id == ID_EXECUTE_POSE) {
        fill = kRedSoft;
        border = kRed;
        text = kRed;
    } else if (id == ID_DRY_RUN) {
        fill = kGreenSoft;
        border = kGreen;
        text = kGreen;
    } else if (id == ID_RECORD || id == ID_PLAYBACK || jog_button) {
        fill = kAccentSoft;
        border = kAccent;
        text = kAccentDark;
    } else if (id == ID_SAFE_POSE) {
        fill = kSurfaceAlt;
        border = kAccent;
        text = kAccentDark;
    }

    if (pressed && !disabled) {
        fill = mix_color(fill, kText, 12);
        border = mix_color(border, kText, 18);
    }

    RECT rect = item.rcItem;
    InflateRect(&rect, -1, -1);
    HBRUSH brush = CreateSolidBrush(fill);
    HPEN pen = CreatePen(PS_SOLID, 1, border);
    HGDIOBJ old_brush = SelectObject(item.hDC, brush);
    HGDIOBJ old_pen = SelectObject(item.hDC, pen);
    const int radius = jog_button ? 12 : 18;
    RoundRect(item.hDC, rect.left, rect.top, rect.right, rect.bottom, radius, radius);
    SelectObject(item.hDC, old_pen);
    SelectObject(item.hDC, old_brush);
    DeleteObject(pen);
    DeleteObject(brush);

    wchar_t label_text[128]{};
    GetWindowTextW(item.hwndItem, label_text, static_cast<int>(std::size(label_text)));
    SetBkMode(item.hDC, TRANSPARENT);
    SetTextColor(item.hDC, text);
    HGDIOBJ old_font = SelectObject(item.hDC, mode_button ? g_app.heading_font : g_app.body_font);
    DrawTextW(item.hDC, label_text, -1, &rect,
              DT_CENTER | DT_VCENTER | DT_SINGLELINE | DT_NOPREFIX);
    SelectObject(item.hDC, old_font);

    if ((item.itemState & ODS_FOCUS) && !disabled) {
        RECT focus = rect;
        InflateRect(&focus, -4, -4);
        DrawFocusRect(item.hDC, &focus);
    }
}

void label(HDC dc, const wchar_t* text, int x, int y, COLORREF color = kMuted) {
    SetTextColor(dc, color);
    SetBkMode(dc, TRANSPARENT);
    TextOutW(dc, x, y, text, static_cast<int>(wcslen(text)));
}

void build_ui(HWND window) {
    g_app.window = window;
    initialize_safety_pose_text();
    g_app.root = std::filesystem::path(L".");
    wchar_t module_path[MAX_PATH]{};
    GetModuleFileNameW(nullptr, module_path, MAX_PATH);
    g_app.root = std::filesystem::path(module_path).parent_path();
    load_logo_image();

    g_app.heading_font = CreateFontW(kHeadingFontHeight, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE,
                                     DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                     CLEARTYPE_QUALITY, FF_SWISS, L"Microsoft YaHei UI");
    g_app.body_font = CreateFontW(kBodyFontHeight, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                                  DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                  CLEARTYPE_QUALITY, FF_SWISS, L"Microsoft YaHei UI");
    g_app.background_brush = CreateSolidBrush(kBackground);
    g_app.surface_brush = CreateSolidBrush(kSurface);
    g_app.surface_alt_brush = CreateSolidBrush(kSurfaceAlt);

    auto add_control = [&](const wchar_t* klass, const wchar_t* text, DWORD style,
                           int x, int y, int width, int height, int id,
                           std::vector<HWND>* group) {
        HWND control = make_control(klass, text, style, x, y, width, height, id);
        if (group && control) group->push_back(control);
        return control;
    };

    g_app.status = add_control(L"STATIC", L"待机",
                               WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE,
                               680, 24, 400, 40, ID_STATUS, nullptr);
    SendMessageW(g_app.status, WM_SETFONT, reinterpret_cast<WPARAM>(g_app.heading_font), TRUE);

    // Control mode card.
    add_control(L"STATIC", L"控制模式", WS_CHILD | WS_VISIBLE,
                40, 102, 200, 24, 0, nullptr);
    add_control(L"BUTTON", L"遥操作", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
                40, 132, 180, 50, ID_MODE_TELEOP, nullptr);
    add_control(L"BUTTON", L"关节控制", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
                232, 132, 180, 50, ID_MODE_JOINT, nullptr);
    add_control(L"BUTTON", L"停止", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
                424, 132, 110, 50, ID_STOP, nullptr);
    add_control(L"BUTTON", L"Dry-run 检查", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
                546, 132, 160, 50, ID_DRY_RUN, nullptr);
    add_control(L"BUTTON", L"真实运动授权（现场确认安全）",
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
                746, 140, 334, 38, ID_ARM_MOTION, nullptr);
    add_control(L"STATIC", L"关节/安全姿态对象", WS_CHILD | WS_VISIBLE | SS_LEFT,
                40, 192, 190, 24, 0, nullptr);
    HWND target_follower = add_control(L"BUTTON", L"跟随臂",
                                        WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_GROUP | BS_AUTORADIOBUTTON,
                                        232, 188, 108, 30, ID_TARGET_FOLLOWER, nullptr);
    add_control(L"BUTTON", L"操作臂",
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTORADIOBUTTON,
                346, 188, 108, 30, ID_TARGET_OPERATOR, nullptr);
    SendMessageW(target_follower, BM_SETCHECK, BST_CHECKED, 0);
    add_control(L"STATIC", L"“关节控制”“轨迹录制”和“轨迹回放”使用该对象；遥操作始终由操作臂拖动并映射到跟随臂。",
                WS_CHILD | WS_VISIBLE | SS_LEFT,
                474, 192, 586, 24, 0, nullptr);

    // Connection and servo settings.
    add_control(L"STATIC", L"连接与伺服参数", WS_CHILD | WS_VISIBLE,
                40, 246, 240, 24, 0, nullptr);
    const std::array<const wchar_t*, 5> setting_labels{
        L"操作臂 IP", L"跟随臂 IP", L"UDP 端口", L"伺服滤波器", L"LPF 截止参数"};
    const std::array<int, 5> setting_x{40, 244, 448, 582, 776};
    const std::array<int, 5> setting_w{190, 190, 120, 180, 180};
    for (int i = 0; i < 5; ++i) {
        add_control(L"STATIC", setting_labels[static_cast<std::size_t>(i)],
                    WS_CHILD | WS_VISIBLE | SS_LEFT,
                    setting_x[static_cast<std::size_t>(i)], 272,
                    setting_w[static_cast<std::size_t>(i)], 20, 0, nullptr);
    }
    add_control(L"EDIT", L"192.168.0.101",
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL,
                40, 292, 190, 32, ID_OPERATOR_IP, nullptr);
    add_control(L"EDIT", L"192.168.0.102",
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL,
                244, 292, 190, 32, ID_FOLLOWER_IP, nullptr);
    add_control(L"EDIT", L"30001",
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL,
                448, 292, 120, 32, ID_PORT, nullptr);
    HWND filter = add_control(L"COMBOBOX", L"",
                              WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | CBS_DROPDOWNLIST,
                              582, 292, 180, 200, ID_FILTER, nullptr);
    SendMessageW(filter, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"none"));
    SendMessageW(filter, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"lpf"));
    SendMessageW(filter, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"nlf"));
    SendMessageW(filter, CB_SETCURSEL, 1, 0);
    add_control(L"EDIT", L"2.5",
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL,
                776, 292, 120, 32, ID_CUTOFF, nullptr);
    g_app.record = add_control(L"BUTTON", L"轨迹录制",
                               WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
                               904, 291, 84, 34, ID_RECORD, nullptr);
    g_app.playback = add_control(L"BUTTON", L"轨迹回放",
                                 WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
                                 994, 291, 86, 34, ID_PLAYBACK, nullptr);

    // Mutually exclusive mode pages.
    add_control(L"STATIC", L"控制操作区", WS_CHILD | WS_VISIBLE,
                40, 376, 220, 24, 0, nullptr);
    HWND idle_title = add_control(L"STATIC", L"等待选择控制模式",
                                  WS_CHILD | WS_VISIBLE, 60, 414, 900, 32, ID_IDLE_TITLE,
                                  &g_app.idle_controls);
    SendMessageW(idle_title, WM_SETFONT, reinterpret_cast<WPARAM>(g_app.heading_font), TRUE);
    add_control(L"STATIC",
                L"1. 勾选真实运动授权并确认现场安全。\r\n"
                L"2. 选择“遥操作”“关节控制”“轨迹录制”或“轨迹回放”。\r\n"
                L"3. 切换模式前请先点击“停止”。",
                WS_CHILD | WS_VISIBLE | SS_LEFT,
                60, 460, 900, 112, ID_IDLE_TEXT, &g_app.idle_controls);

    HWND teleop_title = add_control(L"STATIC", L"遥操作模式已启用",
                                    WS_CHILD, 60, 414, 900, 32, ID_TELEOP_TITLE,
                                    &g_app.teleop_controls);
    SendMessageW(teleop_title, WM_SETFONT, reinterpret_cast<WPARAM>(g_app.heading_font), TRUE);
    add_control(L"STATIC",
                L"在操作臂 192.168.0.101 上进入拖拽模式并保持。\r\n"
                L"跟随臂 192.168.0.102 将按相对关节映射跟随。\r\n"
                L"此模式下关节 +/- 被锁定，避免两种控制方式冲突。",
                WS_CHILD | SS_LEFT, 60, 460, 900, 112, ID_TELEOP_TEXT,
                &g_app.teleop_controls);

    HWND joint_title = add_control(L"STATIC", L"关节控制模式已启用",
                                   WS_CHILD, 60, 414, 900, 32, ID_JOINT_TITLE,
                                   &g_app.joint_controls);
    SendMessageW(joint_title, WM_SETFONT, reinterpret_cast<WPARAM>(g_app.heading_font), TRUE);
    add_control(L"STATIC", L"按住 +/- 点动对应关节，松开后立即停止。此模式忽略操作臂拖拽跟随。",
                WS_CHILD | SS_LEFT, 60, 444, 1000, 60, ID_JOINT_TEXT,
                &g_app.joint_controls);

    const std::array<const wchar_t*, 6> joints{L"J1", L"J2", L"J3", L"J4", L"J5", L"J6"};
    const std::array<int, 12> ids{ID_J1_MINUS, ID_J1_PLUS, ID_J2_MINUS, ID_J2_PLUS,
                                  ID_J3_MINUS, ID_J3_PLUS, ID_J4_MINUS, ID_J4_PLUS,
                                  ID_J5_MINUS, ID_J5_PLUS, ID_J6_MINUS, ID_J6_PLUS};
    for (int i = 0; i < 6; ++i) {
        const int x = 60 + (i % 3) * 350;
        const int y = 510 + (i / 3) * 68;
        add_control(L"STATIC", joints[static_cast<std::size_t>(i)],
                    WS_CHILD | SS_CENTER | SS_CENTERIMAGE, x, y, 38, 40, 0,
                    &g_app.joint_controls);
        HWND minus = add_control(L"BUTTON", L"-", WS_CHILD | WS_TABSTOP | BS_OWNERDRAW | WS_DISABLED,
                                 x + 48, y, 82, 40, ids[static_cast<std::size_t>(i * 2)],
                                 &g_app.joint_controls);
        HWND plus = add_control(L"BUTTON", L"+", WS_CHILD | WS_TABSTOP | BS_OWNERDRAW | WS_DISABLED,
                                x + 140, y, 82, 40, ids[static_cast<std::size_t>(i * 2 + 1)],
                                &g_app.joint_controls);
        SetWindowSubclass(minus, jog_button_subclass, static_cast<UINT_PTR>(i * 2), static_cast<DWORD_PTR>(i * 2));
        SetWindowSubclass(plus, jog_button_subclass, static_cast<UINT_PTR>(i * 2 + 1), static_cast<DWORD_PTR>(i * 2 + 1));
    }

    // Safety pose and maintenance.
    // The six edit boxes show the selected target's independent parameter set.
    g_app.pose_title = add_control(L"STATIC", L"安全姿态参数", WS_CHILD | WS_VISIBLE,
                                   40, 656, 700, 24, 0, nullptr);
    const auto& pose_ids = pose_control_ids();
    for (int i = 0; i < 6; ++i) {
        const int x = 40 + i * 115;
        std::wstring title = L"J" + std::to_wstring(i + 1);
        add_control(L"STATIC", title.c_str(), WS_CHILD | WS_VISIBLE | SS_CENTER,
                    x, 686, 24, 28, 0, nullptr);
        add_control(L"EDIT", L"",
                    WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL,
                    x + 26, 682, 78, 30, pose_ids[static_cast<std::size_t>(i)], nullptr);
    }
    // Load the follower's defaults first; operator defaults remain separate.
    load_safety_pose_text(g_app.joint_target);
    add_control(L"BUTTON", L"预览安全姿态", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
                780, 680, 130, 38, ID_SAFE_POSE, nullptr);
    add_control(L"BUTTON", L"执行安全姿态", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW | WS_DISABLED,
                922, 680, 138, 38, ID_EXECUTE_POSE, nullptr);

    // Log card.
    add_control(L"STATIC", L"实时会话日志", WS_CHILD | WS_VISIBLE,
                40, 790, 240, 24, 0, nullptr);
    g_app.log = add_control(L"EDIT", L"",
                            WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_MULTILINE |
                            ES_READONLY | WS_VSCROLL | ES_AUTOVSCROLL,
                            40, 818, 1020, 62, ID_LOG, nullptr);
    SendMessageW(g_app.log, EM_SETLIMITTEXT, 20000, 0);

    set_status(L"待机", kMuted);
    update_mode_ui();
    append_log(L"JAKA 双臂遥操作控制台");
    append_log(L"遥操作与关节控制已设为互斥模式；切换前请先停止当前会话");
}
void layout_resizable_controls(HWND window) {
    if (!g_app.status || !g_app.log) return;

    RECT client{};
    if (!GetClientRect(window, &client)) return;
    const int client_width = client.right - client.left;
    const int client_height = client.bottom - client.top;

    const int status_width = std::max(360, client_width - 744);
    SetWindowPos(g_app.status, nullptr, 620, 24, status_width, 40,
                 SWP_NOZORDER | SWP_NOACTIVATE | SWP_NOOWNERZORDER);

    const int log_width = std::max(1020, client_width - 80);
    const int log_height = std::max(62, client_height - 858);
    SetWindowPos(g_app.log, nullptr, 40, 818, log_width, log_height,
                 SWP_NOZORDER | SWP_NOACTIVATE | SWP_NOOWNERZORDER);

    InvalidateRect(window, nullptr, TRUE);
}
LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    switch (message) {
    case WM_CREATE:
        build_ui(window);
        SetTimer(window, kUiTimer, 250, nullptr);
        SetTimer(window, kJogTimer, 8, nullptr);
        return 0;
    case WM_GETMINMAXINFO: {
        auto* limits = reinterpret_cast<MINMAXINFO*>(lparam);
        limits->ptMinTrackSize.x = kMinimumWindowWidth;
        limits->ptMinTrackSize.y = kMinimumWindowHeight;
        return 0;
    }
    case WM_SIZE:
        if (wparam != SIZE_MINIMIZED) layout_resizable_controls(window);
        return 0;
    case WM_TIMER:
        if (wparam == kUiTimer && g_app.child_process && !process_running()) {
            drain_child_output();
            append_log(L"会话进程已退出");
            close_child_handles();
            g_app.real_session = false;
            g_app.mode = WorkMode::Idle;
            g_app.jog_axis = -1;
            g_app.jog_delta = 0.0;
            set_status(L"未连接", kMuted);
            update_mode_ui();
        } else if (wparam == kUiTimer) {
            drain_child_output();
        } else if (wparam == kJogTimer && g_app.jog_axis >= 0 && process_running()) {
            send_pipe_line("JOG " + std::to_string(g_app.jog_axis) + " " + std::to_string(g_app.jog_delta));
        }
        return 0;
    case WM_COMMAND: {
        const int id = LOWORD(wparam);
        if (id == ID_MODE_TELEOP && HIWORD(wparam) == BN_CLICKED) {
            start_child(WorkMode::Teleop);
        } else if (id == ID_MODE_JOINT && HIWORD(wparam) == BN_CLICKED) {
            start_child(WorkMode::Joint);
        } else if (id == ID_DRY_RUN && HIWORD(wparam) == BN_CLICKED) {
            start_child(WorkMode::DryRun);
        } else if ((id == ID_TARGET_FOLLOWER || id == ID_TARGET_OPERATOR) &&
                   HIWORD(wparam) == BN_CLICKED) {
            capture_safety_pose_text(g_app.joint_target);
            g_app.joint_target = id == ID_TARGET_OPERATOR ? ControlTarget::Operator
                                                          : ControlTarget::Follower;
            load_safety_pose_text(g_app.joint_target);
            update_mode_ui();
            append_log(std::wstring(L"关节/安全姿态对象已切换为 ") +
                       control_target_label(g_app.joint_target));
        } else if (id == ID_STOP && HIWORD(wparam) == BN_CLICKED) {
            stop_child();
        } else if (id == ID_RECORD && HIWORD(wparam) == BN_CLICKED) {
            toggle_recording();
        } else if (id == ID_PLAYBACK && HIWORD(wparam) == BN_CLICKED) {
            choose_and_playback();
        } else if (id == ID_SAFE_POSE && HIWORD(wparam) == BN_CLICKED) {
            preview_pose();
        } else if (id == ID_EXECUTE_POSE && HIWORD(wparam) == BN_CLICKED) {
            execute_pose();
        } else if (id == ID_ARM_MOTION && HIWORD(wparam) == BN_CLICKED) {
            const bool armed = SendMessageW(GetDlgItem(window, ID_ARM_MOTION), BM_GETCHECK, 0, 0) == BST_CHECKED;
            update_mode_ui();
            set_status(armed ? L"真实运动已授权 / 请选择控制模式" : L"Dry-run 就绪",
                       armed ? kAccentDark : kMuted);
            append_log(armed ? L"真实运动授权已开启，请选择“遥操作”或“关节控制”"
                              : L"真实运动授权已取消，遥操作和关节控制已锁定");
        }
        else if (id >= ID_J1_MINUS && id <= ID_J6_PLUS && HIWORD(wparam) == BN_CLICKED) {
            append_log(L"请按住关节按钮进行点动，松开后停止");
        }
        return 0;
    }
    case WM_CTLCOLORSTATIC:
    case WM_CTLCOLOREDIT:
    case WM_CTLCOLORLISTBOX:
    case WM_CTLCOLORBTN: {
        HDC dc = reinterpret_cast<HDC>(wparam);
        if (message == WM_CTLCOLORSTATIC) {
            const bool is_status = reinterpret_cast<HWND>(lparam) == g_app.status;
            // Dynamic STATIC controls must erase their previous text. Returning
            // a hollow brush leaves stale glyphs behind when labels change
            // between Idle, Teleop, Joint, and Dry-run pages.
            SetBkMode(dc, OPAQUE);
            SetBkColor(dc, is_status ? kBackground : kSurface);
            SetTextColor(dc, is_status ? g_app.status_color : kText);
            return reinterpret_cast<LRESULT>(is_status ? g_app.background_brush : g_app.surface_brush);
        }
        SetBkColor(dc, kSurface);
        SetTextColor(dc, kText);
        return reinterpret_cast<LRESULT>(g_app.surface_brush);
    }
    case WM_DRAWITEM: {
        const auto* item = reinterpret_cast<const DRAWITEMSTRUCT*>(lparam);
        if (item && item->CtlType == ODT_BUTTON) {
            draw_owner_button(*item);
            return TRUE;
        }
        break;
    }
    case WM_ERASEBKGND: {
        RECT rect{};
        GetClientRect(window, &rect);
        FillRect(reinterpret_cast<HDC>(wparam), &rect, g_app.background_brush);
        return 1;
    }
    case WM_PAINT: {
        PAINTSTRUCT paint{};
        HDC dc = BeginPaint(window, &paint);
        RECT client{};
        GetClientRect(window, &client);
        const int width = client.right - client.left;
        const int height = client.bottom - client.top;
        const int card_width = std::max(1080, width - 40);
        const int log_card_height = std::max(118, height - 802);
        HDC buffer = CreateCompatibleDC(dc);
        HBITMAP bitmap = buffer ? CreateCompatibleBitmap(dc, width, height) : nullptr;
        if (buffer && bitmap) {
            HGDIOBJ old_bitmap = SelectObject(buffer, bitmap);
            FillRect(buffer, &client, g_app.background_brush);
            draw_section(buffer, 20, 90, card_width, 140);
            draw_section(buffer, 20, 232, card_width, 120);
            draw_section(buffer, 20, 360, card_width, 280);
            draw_section(buffer, 20, 642, card_width, 94);
            draw_section(buffer, 20, 776, card_width, log_card_height);
            SetBkMode(buffer, TRANSPARENT);
            SetTextColor(buffer, kText);
            SelectObject(buffer, g_app.heading_font);
            RECT title_rect{32, 18, 600, 50};
            DrawTextW(buffer, L"JAKA 双臂遥操作控制台", -1, &title_rect,
                      DT_LEFT | DT_TOP | DT_SINGLELINE | DT_NOPREFIX);
            SelectObject(buffer, g_app.body_font);
            label(buffer, L"拖拽跟随 / 关节点动 / 关节轨迹录制与回放 / Dry-run 安全检查", 32, 58, kAccentDark);
            draw_logo(buffer, width);
            BitBlt(dc, 0, 0, width, height, buffer, 0, 0, SRCCOPY);
            SelectObject(buffer, old_bitmap);
        } else {
            FillRect(dc, &client, g_app.background_brush);
            draw_section(dc, 20, 90, card_width, 140);
            draw_section(dc, 20, 232, card_width, 120);
            draw_section(dc, 20, 360, card_width, 280);
            draw_section(dc, 20, 642, card_width, 94);
            draw_section(dc, 20, 776, card_width, log_card_height);
            SetBkMode(dc, TRANSPARENT);
            SetTextColor(dc, kText);
            SelectObject(dc, g_app.heading_font);
            RECT title_rect{32, 18, 600, 50};
            DrawTextW(dc, L"JAKA 双臂遥操作控制台", -1, &title_rect,
                      DT_LEFT | DT_TOP | DT_SINGLELINE | DT_NOPREFIX);
            SelectObject(dc, g_app.body_font);
            label(dc, L"拖拽跟随 / 关节点动 / 关节轨迹录制与回放 / Dry-run 安全检查", 32, 58, kAccentDark);
            draw_logo(dc, width);
        }
        if (bitmap) DeleteObject(bitmap);
        if (buffer) DeleteDC(buffer);
        EndPaint(window, &paint);
        return 0;
    }
    case WM_DESTROY:
        stop_child();
        KillTimer(window, kUiTimer);
        KillTimer(window, kJogTimer);
        DeleteObject(g_app.heading_font);
        DeleteObject(g_app.body_font);
        DeleteObject(g_app.background_brush);
        DeleteObject(g_app.surface_brush);
        DeleteObject(g_app.surface_alt_brush);
        PostQuitMessage(0);
        return 0;
    default:
        break;
    }
    return DefWindowProcW(window, message, wparam, lparam);
}

}  // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show_command) {
    INITCOMMONCONTROLSEX common_controls{sizeof(common_controls), ICC_STANDARD_CLASSES};
    InitCommonControlsEx(&common_controls);

    Gdiplus::GdiplusStartupInput gdiplus_input;
    ULONG_PTR gdiplus_token = 0;
    if (Gdiplus::GdiplusStartup(&gdiplus_token, &gdiplus_input, nullptr) != Gdiplus::Ok) return 10;

    WNDCLASSW window_class{};
    window_class.style = CS_HREDRAW | CS_VREDRAW;
    window_class.hInstance = instance;
    window_class.lpfnWndProc = window_proc;
    window_class.lpszClassName = L"JakaDualTeleopGui";
    window_class.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    window_class.hbrBackground = g_app.background_brush;
    if (!RegisterClassW(&window_class)) return 10;

    HWND window = CreateWindowExW(0, window_class.lpszClassName, L"JAKA 双臂遥操作",
                                 WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX |
                                     WS_MAXIMIZEBOX | WS_THICKFRAME | WS_CLIPCHILDREN,
                                 CW_USEDEFAULT, CW_USEDEFAULT, kInitialWindowWidth, kInitialWindowHeight,
                                 nullptr, nullptr, instance, nullptr);
    if (!window) {
        Gdiplus::GdiplusShutdown(gdiplus_token);
        return 10;
    }
    ShowWindow(window, show_command);
    UpdateWindow(window);

    MSG message{};
    while (GetMessageW(&message, nullptr, 0, 0) > 0) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }

    delete g_app.logo_image;
    g_app.logo_image = nullptr;
    Gdiplus::GdiplusShutdown(gdiplus_token);
    return static_cast<int>(message.wParam);
}







