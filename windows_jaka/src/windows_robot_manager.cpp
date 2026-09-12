#define UNICODE
#define _UNICODE
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commctrl.h>
#include <commdlg.h>

#include "control_pipe.hpp"
#include "robot_registry.hpp"
#include "runtime_plan.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <memory>
#include <set>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "comdlg32.lib")

namespace {

constexpr COLORREF kBackground = RGB(248, 250, 252);
constexpr COLORREF kSurface = RGB(255, 255, 255);
constexpr COLORREF kSurfaceMuted = RGB(241, 245, 249);
constexpr COLORREF kBorder = RGB(226, 232, 240);
constexpr COLORREF kText = RGB(51, 65, 85);
constexpr COLORREF kMuted = RGB(71, 85, 105);
constexpr COLORREF kPrimary = RGB(51, 65, 85);
constexpr COLORREF kAccent = RGB(234, 88, 12);
constexpr COLORREF kSuccess = RGB(5, 150, 105);
constexpr int kHeadingFontHeight = 24;
constexpr int kBodyFontHeight = 18;
constexpr int kSmallFontHeight = 16;

constexpr int ID_ROBOT_LIST = 1000;
constexpr int ID_GROUP_LIST = 1001;
constexpr int ID_STATUS = 1002;

constexpr int ID_ROBOT_ID = 1100;
constexpr int ID_ROBOT_NAME = 1101;
constexpr int ID_ROBOT_MODEL = 1102;
constexpr int ID_ROBOT_IP = 1103;
constexpr int ID_ROBOT_ENABLED = 1104;
constexpr int ID_ROBOT_FILTER = 1105;
constexpr int ID_ROBOT_LPF = 1106;
constexpr int ID_ROBOT_VEL = 1107;
constexpr int ID_ROBOT_ACC = 1108;
constexpr int ID_ROBOT_DIRECTION = 1109;
constexpr int ID_ROBOT_LOWER = 1110;
constexpr int ID_ROBOT_UPPER = 1111;
constexpr int ID_ROBOT_SAFE = 1112;
constexpr int ID_ROBOT_ADD = 1113;
constexpr int ID_ROBOT_APPLY = 1114;
constexpr int ID_ROBOT_DELETE = 1115;

constexpr int ID_GROUP_ID = 1200;
constexpr int ID_GROUP_NAME = 1201;
constexpr int ID_GROUP_OPERATOR = 1202;
constexpr int ID_GROUP_FOLLOWERS = 1203;
constexpr int ID_GROUP_ADD = 1204;
constexpr int ID_GROUP_APPLY = 1205;
constexpr int ID_GROUP_DELETE = 1206;

constexpr int ID_SAVE = 1300;
constexpr int ID_RELOAD = 1301;
constexpr int ID_GROUP_REAL_MOTION = 1302;
constexpr int ID_GROUP_START = 1303;
constexpr int ID_GROUP_STOP = 1304;
constexpr int ID_SESSION_STATUS = 1305;
constexpr int ID_STATUS_LIST = 1306;
constexpr int ID_SINGLE_JOINT = 1400;
constexpr int ID_SINGLE_RECORD = 1401;
constexpr int ID_SINGLE_PLAYBACK = 1402;
constexpr int ID_SINGLE_STOP = 1403;
constexpr int ID_SAFE_EXECUTE = 1404;
constexpr int ID_STOP_ALL = 1405;
constexpr int ID_JOG_BASE = 1500;
constexpr int ID_OPEN_STATUS_PAGE = 1600;
constexpr int ID_STATUS_BACK = 1601;
constexpr int ID_STATUS_PAGE_PRIMARY = 1602;
constexpr int ID_STATUS_PAGE_DIAG = 1603;
constexpr int ID_STATUS_PAGE_SUMMARY = 1604;
constexpr UINT_PTR ID_SESSION_TIMER = 1;

HWND g_window{};
HWND g_status{};
HWND g_robot_list{};
HWND g_group_list{};
HWND g_session_status{};
HWND g_status_list{};
HWND g_status_page_primary{};
HWND g_status_page_diag{};
HWND g_status_page_summary{};
std::vector<HWND> g_status_page_controls;
bool g_status_page_open{false};
HFONT g_heading_font{};
HFONT g_body_font{};
HFONT g_small_font{};
HBRUSH g_background_brush{};
HBRUSH g_surface_brush{};
std::filesystem::path g_status_directory;
enum class SessionKind { Group, Single };
struct ActiveSession {
    std::string key;
    std::string label;
    SessionKind kind{SessionKind::Group};
    HANDLE process{};
    HANDLE job{};
    std::wstring control_pipe;
    std::vector<std::wstring> stop_pipes;
    std::vector<std::string> robot_ids;
    std::string mode;
    std::uint16_t base_port{0};
    std::filesystem::path log_path;
    ~ActiveSession() {
        if (job) CloseHandle(job);
        if (process) CloseHandle(process);
    }
};
std::vector<std::unique_ptr<ActiveSession>> g_sessions;
int g_jog_axis{-1};
double g_jog_delta{0.0};
windows_jaka::RobotRegistry g_registry;
std::filesystem::path g_registry_path;
std::vector<HWND> g_robot_controls;
std::vector<HWND> g_group_controls;
bool g_suppress_selection = false;

std::wstring widen(const std::string& text) {
    if (text.empty()) return {};
    const int size = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0);
    std::wstring output(static_cast<std::size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), output.data(), size);
    return output;
}

std::string narrow(const std::wstring& text) {
    if (text.empty()) return {};
    const int size = WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()),
                                         nullptr, 0, nullptr, nullptr);
    std::string output(static_cast<std::size_t>(size), '\0');
    WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()),
                        output.data(), size, nullptr, nullptr);
    return output;
}

std::string read_text(HWND control) {
    const int length = GetWindowTextLengthW(control);
    if (length <= 0) return {};
    std::wstring text(static_cast<std::size_t>(length) + 1, L'\0');
    GetWindowTextW(control, text.data(), length + 1);
    text.resize(static_cast<std::size_t>(length));
    return narrow(text);
}

void set_text(HWND control, const std::string& text) {
    SetWindowTextW(control, widen(text).c_str());
}

void set_status(const std::wstring& text) {
    SetWindowTextW(g_status, text.c_str());
}

bool parse_numbers(const std::string& text, windows_jaka::JointArray& values, double scale = 1.0) {
    std::stringstream input(text);
    std::string item;
    for (int i = 0; i < 6; ++i) {
        if (!std::getline(input, item, ',')) return false;
        try {
            std::size_t consumed = 0;
            const double value = std::stod(item, &consumed);
            while (consumed < item.size() && std::isspace(static_cast<unsigned char>(item[consumed]))) ++consumed;
            if (consumed != item.size() || !std::isfinite(value)) return false;
            values[static_cast<std::size_t>(i)] = value * scale;
        } catch (...) {
            return false;
        }
    }
    std::string trailing;
    return !std::getline(input, trailing, ',');
}

std::string format_numbers(const windows_jaka::JointArray& values, double scale = 1.0) {
    std::ostringstream output;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) output << ',';
        output << values[i] * scale;
    }
    return output.str();
}

HWND add_control(HWND parent, const wchar_t* klass, const wchar_t* text, DWORD style,
                 int x, int y, int width, int height, int id,
                 std::vector<HWND>* group = nullptr) {
    HWND control = CreateWindowExW(0, klass, text, style | WS_CHILD | WS_VISIBLE,
                                   x, y, width, height, parent,
                                   reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
                                   GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT,
                 reinterpret_cast<WPARAM>(g_body_font ? g_body_font : GetStockObject(DEFAULT_GUI_FONT)), TRUE);
    if (group) group->push_back(control);
    return control;
}

void add_label(HWND parent, const wchar_t* text, int x, int y, int width,
               std::vector<HWND>* group = nullptr) {
    add_control(parent, L"STATIC", text, SS_LEFT, x, y, width, 24, 0, group);
}

void add_section_title(HWND parent, const wchar_t* text, int x, int y, int width) {
    HWND control = CreateWindowExW(0, L"STATIC", text, WS_CHILD | WS_VISIBLE | SS_LEFT,
                                   x, y, width, 32, parent, nullptr,
                                   GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT,
                 reinterpret_cast<WPARAM>(g_heading_font ? g_heading_font : GetStockObject(DEFAULT_GUI_FONT)), TRUE);
}

void add_group_box(HWND parent, const wchar_t* text, int x, int y, int width, int height) {
    HWND box = CreateWindowExW(0, L"BUTTON", text, WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                               x, y, width, height, parent, nullptr,
                               GetModuleHandleW(nullptr), nullptr);
    SendMessageW(box, WM_SETFONT,
                 reinterpret_cast<WPARAM>(g_small_font ? g_small_font : GetStockObject(DEFAULT_GUI_FONT)), TRUE);
}

void add_column(HWND list, int index, int width, const wchar_t* title) {
    LVCOLUMNW column{};
    column.mask = LVCF_TEXT | LVCF_WIDTH;
    column.cx = width;
    column.pszText = const_cast<wchar_t*>(title);
    ListView_InsertColumn(list, index, &column);
}

void fill_robot_list();
void fill_group_list();
void load_robot_form(int index);
void load_group_form(int index);
bool session_running();
void start_selected_group();
void stop_selected_group();
void stop_selected_single();
void stop_all_sessions();
void update_session_ui();
void refresh_status_list();
void refresh_status_page();
void show_main_page();
void show_status_page();
void start_single_session(const std::string& control_mode);
bool send_single_pipe_line(const std::string& line);
LRESULT CALLBACK jog_button_subclass(HWND hwnd, UINT message, WPARAM wparam,
                                     LPARAM lparam, UINT_PTR, DWORD_PTR data);
void stop_selected_group();
std::filesystem::path workdir_root();

void add_edit(HWND parent, int id, int x, int y, int width,
              std::vector<HWND>* group = nullptr) {
    add_control(parent, L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL,
                x, y, width, 28, id, group);
}

void build_ui(HWND window) {
    g_heading_font = CreateFontW(kHeadingFontHeight, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE,
                                  DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                  CLEARTYPE_QUALITY, FF_SWISS, L"Microsoft YaHei UI");
    g_body_font = CreateFontW(kBodyFontHeight, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                              DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                              CLEARTYPE_QUALITY, FF_SWISS, L"Microsoft YaHei UI");
    g_small_font = CreateFontW(kSmallFontHeight, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                               DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                               CLEARTYPE_QUALITY, FF_SWISS, L"Microsoft YaHei UI");
    g_background_brush = CreateSolidBrush(kBackground);
    g_surface_brush = CreateSolidBrush(kSurface);

    add_section_title(window, L"JAKA 多机器人控制台", 24, 16, 520);
    add_control(window, L"STATIC", L"设备管理  /  单臂控制  /  一拖多遥操作  /  实时诊断",
                SS_LEFT, 26, 52, 620, 26, 0, nullptr);
    g_status = CreateWindowExW(0, L"STATIC", L"", WS_CHILD | WS_VISIBLE | SS_RIGHT | SS_CENTERIMAGE,
                               700, 20, 570, 40, window,
                               reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_STATUS)),
                               GetModuleHandleW(nullptr), nullptr);
    SendMessageW(g_status, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);
    add_control(window, L"BUTTON", L"实时状态", WS_TABSTOP | BS_PUSHBUTTON,
                1110, 20, 140, 40, ID_OPEN_STATUS_PAGE, nullptr);

    add_group_box(window, L"机器人设备与参数", 16, 84, 1228, 462);
    add_label(window, L"机器人列表", 34, 106, 260);
    g_robot_list = CreateWindowExW(WS_EX_CLIENTEDGE, WC_LISTVIEWW, L"",
                                   WS_CHILD | WS_VISIBLE | WS_TABSTOP | LVS_REPORT | LVS_SINGLESEL,
                                   34, 134, 350, 340, window,
                                   reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_ROBOT_LIST)),
                                   GetModuleHandleW(nullptr), nullptr);
    SendMessageW(g_robot_list, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);
    ListView_SetExtendedListViewStyle(g_robot_list,
                                      LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES | LVS_EX_DOUBLEBUFFER | LVS_EX_LABELTIP);
    add_column(g_robot_list, 0, 90, L"ID");
    add_column(g_robot_list, 1, 120, L"名称");
    add_column(g_robot_list, 2, 120, L"IP");

    add_label(window, L"参数编辑", 420, 106, 260);
    add_label(window, L"ID", 420, 142, 80);
    add_edit(window, ID_ROBOT_ID, 510, 138, 220, &g_robot_controls);
    add_label(window, L"名称", 770, 142, 80);
    add_edit(window, ID_ROBOT_NAME, 850, 138, 350, &g_robot_controls);
    add_label(window, L"型号", 420, 182, 80);
    add_edit(window, ID_ROBOT_MODEL, 510, 178, 220, &g_robot_controls);
    add_label(window, L"IP 地址", 770, 182, 80);
    add_edit(window, ID_ROBOT_IP, 850, 178, 350, &g_robot_controls);
    add_control(window, L"BUTTON", L"启用该机器人", BS_AUTOCHECKBOX,
                420, 220, 180, 30, ID_ROBOT_ENABLED, &g_robot_controls);
    add_label(window, L"滤波器", 620, 224, 80, &g_robot_controls);
    HWND filter = CreateWindowExW(0, L"COMBOBOX", L"",
                                  WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST,
                                  710, 218, 140, 220, window,
                                  reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_ROBOT_FILTER)),
                                  GetModuleHandleW(nullptr), nullptr);
    SendMessageW(filter, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);
    SendMessageW(filter, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"none"));
    SendMessageW(filter, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"lpf"));
    SendMessageW(filter, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"nlf"));
    SendMessageW(filter, CB_SETCURSEL, 1, 0);
    g_robot_controls.push_back(filter);
    add_label(window, L"LPF 截止", 880, 224, 90, &g_robot_controls);
    add_edit(window, ID_ROBOT_LPF, 970, 218, 110, &g_robot_controls);

    add_label(window, L"最大速度 rad/s", 420, 266, 150, &g_robot_controls);
    add_edit(window, ID_ROBOT_VEL, 570, 260, 150, &g_robot_controls);
    add_label(window, L"最大加速度 rad/s²", 750, 266, 170, &g_robot_controls);
    add_edit(window, ID_ROBOT_ACC, 920, 260, 160, &g_robot_controls);
    add_label(window, L"关节方向（+, -）", 420, 308, 170, &g_robot_controls);
    add_edit(window, ID_ROBOT_DIRECTION, 590, 302, 610, &g_robot_controls);
    add_label(window, L"关节下限（角度）", 420, 350, 170, &g_robot_controls);
    add_edit(window, ID_ROBOT_LOWER, 590, 344, 610, &g_robot_controls);
    add_label(window, L"关节上限（角度）", 420, 392, 170, &g_robot_controls);
    add_edit(window, ID_ROBOT_UPPER, 590, 386, 610, &g_robot_controls);
    add_label(window, L"安全姿态（角度）", 420, 434, 170, &g_robot_controls);
    add_edit(window, ID_ROBOT_SAFE, 590, 428, 610, &g_robot_controls);

    add_control(window, L"BUTTON", L"新增机器人", WS_TABSTOP | BS_PUSHBUTTON,
                34, 490, 120, 38, ID_ROBOT_ADD, &g_robot_controls);
    add_control(window, L"BUTTON", L"应用修改", WS_TABSTOP | BS_PUSHBUTTON,
                162, 490, 120, 38, ID_ROBOT_APPLY, &g_robot_controls);
    add_control(window, L"BUTTON", L"删除选中", WS_TABSTOP | BS_PUSHBUTTON,
                290, 490, 120, 38, ID_ROBOT_DELETE, &g_robot_controls);
    add_control(window, L"BUTTON", L"保存配置", WS_TABSTOP | BS_DEFPUSHBUTTON,
                1080, 490, 140, 38, ID_SAVE, &g_robot_controls);
    add_control(window, L"BUTTON", L"重新加载", WS_TABSTOP | BS_PUSHBUTTON,
                940, 490, 130, 38, ID_RELOAD, &g_robot_controls);

    add_group_box(window, L"遥操作组与运行控制", 16, 552, 1228, 270);
    add_label(window, L"遥操作组", 34, 570, 260);
    g_group_list = CreateWindowExW(WS_EX_CLIENTEDGE, WC_LISTVIEWW, L"",
                                   WS_CHILD | WS_VISIBLE | WS_TABSTOP | LVS_REPORT | LVS_SINGLESEL,
                                   34, 598, 350, 150, window,
                                   reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_GROUP_LIST)),
                                   GetModuleHandleW(nullptr), nullptr);
    SendMessageW(g_group_list, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);
    ListView_SetExtendedListViewStyle(g_group_list,
                                      LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES | LVS_EX_DOUBLEBUFFER);
    add_column(g_group_list, 0, 100, L"组 ID");
    add_column(g_group_list, 1, 220, L"名称");

    add_label(window, L"组参数", 420, 570, 200, &g_group_controls);
    add_label(window, L"组 ID", 420, 604, 90, &g_group_controls);
    add_edit(window, ID_GROUP_ID, 510, 598, 220, &g_group_controls);
    add_label(window, L"组名称", 770, 604, 90, &g_group_controls);
    add_edit(window, ID_GROUP_NAME, 860, 598, 340, &g_group_controls);
    add_label(window, L"操作臂 ID", 420, 642, 120, &g_group_controls);
    add_edit(window, ID_GROUP_OPERATOR, 540, 636, 190, &g_group_controls);
    add_label(window, L"跟随臂 ID（逗号分隔）", 770, 642, 220, &g_group_controls);
    add_edit(window, ID_GROUP_FOLLOWERS, 990, 636, 210, &g_group_controls);

    add_control(window, L"BUTTON", L"新增组", WS_TABSTOP | BS_PUSHBUTTON,
                34, 762, 110, 36, ID_GROUP_ADD, &g_group_controls);
    add_control(window, L"BUTTON", L"应用组修改", WS_TABSTOP | BS_PUSHBUTTON,
                152, 762, 120, 36, ID_GROUP_APPLY, &g_group_controls);
    add_control(window, L"BUTTON", L"删除组", WS_TABSTOP | BS_PUSHBUTTON,
                280, 762, 104, 36, ID_GROUP_DELETE, &g_group_controls);
    add_control(window, L"BUTTON", L"真实运动授权（组/单台）", BS_AUTOCHECKBOX,
                420, 682, 260, 30, ID_GROUP_REAL_MOTION, &g_group_controls);
    add_control(window, L"BUTTON", L"启动选中组", WS_TABSTOP | BS_DEFPUSHBUTTON,
                700, 678, 130, 38, ID_GROUP_START, &g_group_controls);
    add_control(window, L"BUTTON", L"停止当前组", WS_TABSTOP | BS_PUSHBUTTON,
                840, 678, 130, 38, ID_GROUP_STOP, &g_group_controls);
    add_control(window, L"BUTTON", L"全部停止", WS_TABSTOP | BS_PUSHBUTTON,
                980, 678, 110, 38, ID_STOP_ALL, &g_group_controls);
    g_session_status = CreateWindowExW(0, L"STATIC", L"无运行会话",
                                       WS_CHILD | WS_VISIBLE | SS_LEFT | SS_CENTERIMAGE,
                                       1100, 678, 118, 38, window,
                                       reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_SESSION_STATUS)),
                                       GetModuleHandleW(nullptr), nullptr);
    SendMessageW(g_session_status, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);

    add_label(window, L"单台机器人控制", 420, 726, 180, &g_robot_controls);
    add_control(window, L"BUTTON", L"关节控制", WS_TABSTOP | BS_PUSHBUTTON,
                570, 718, 100, 36, ID_SINGLE_JOINT, &g_robot_controls);
    add_control(window, L"BUTTON", L"轨迹录制", WS_TABSTOP | BS_PUSHBUTTON,
                680, 718, 100, 36, ID_SINGLE_RECORD, &g_robot_controls);
    add_control(window, L"BUTTON", L"轨迹回放", WS_TABSTOP | BS_PUSHBUTTON,
                790, 718, 100, 36, ID_SINGLE_PLAYBACK, &g_robot_controls);
    add_control(window, L"BUTTON", L"停止单台", WS_TABSTOP | BS_PUSHBUTTON,
                900, 718, 100, 36, ID_SINGLE_STOP, &g_robot_controls);
    add_control(window, L"BUTTON", L"执行安全姿态", WS_TABSTOP | BS_PUSHBUTTON,
                1010, 718, 130, 36, ID_SAFE_EXECUTE, &g_robot_controls);

    for (int axis = 0; axis < 6; ++axis) {
        const int x = 420 + axis * 125;
        std::wstring joint = L"J" + std::to_wstring(axis + 1);
        add_control(window, L"STATIC", joint.c_str(), SS_LEFT,
                    x, 768, 34, 26, 0, &g_robot_controls);
        HWND minus = CreateWindowExW(0, L"BUTTON", L"-", WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                                     x + 40, 762, 36, 32, window,
                                     reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_JOG_BASE + axis * 2)),
                                     GetModuleHandleW(nullptr), nullptr);
        HWND plus = CreateWindowExW(0, L"BUTTON", L"+", WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                                    x + 80, 762, 36, 32, window,
                                    reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_JOG_BASE + axis * 2 + 1)),
                                    GetModuleHandleW(nullptr), nullptr);
        SendMessageW(minus, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);
        SendMessageW(plus, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);
        SetWindowSubclass(minus, jog_button_subclass, static_cast<UINT_PTR>(axis * 2), static_cast<DWORD_PTR>(axis * 2));
        SetWindowSubclass(plus, jog_button_subclass, static_cast<UINT_PTR>(axis * 2 + 1), static_cast<DWORD_PTR>(axis * 2 + 1));
        g_robot_controls.push_back(minus);
        g_robot_controls.push_back(plus);
    }

    add_group_box(window, L"机器人实时状态", 16, 828, 1228, 168);
    g_status_list = CreateWindowExW(WS_EX_CLIENTEDGE, WC_LISTVIEWW, L"",
                                    WS_CHILD | WS_VISIBLE | LVS_REPORT | LVS_SINGLESEL,
                                    34, 858, 1192, 122, window,
                                    reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_STATUS_LIST)),
                                    GetModuleHandleW(nullptr), nullptr);
    SendMessageW(g_status_list, WM_SETFONT, reinterpret_cast<WPARAM>(g_small_font), TRUE);
    ListView_SetExtendedListViewStyle(g_status_list,
                                      LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES | LVS_EX_DOUBLEBUFFER | LVS_EX_LABELTIP);
    add_column(g_status_list, 0, 110, L"机器人");
    add_column(g_status_list, 1, 90, L"模式");
    add_column(g_status_list, 2, 55, L"连接");
    add_column(g_status_list, 3, 55, L"上电");
    add_column(g_status_list, 4, 55, L"使能");
    add_column(g_status_list, 5, 55, L"拖动");
    add_column(g_status_list, 6, 55, L"伺服");
    add_column(g_status_list, 7, 120, L"故障");
    add_column(g_status_list, 8, 70, L"年龄ms");
    add_column(g_status_list, 9, 65, L"序列");
    add_column(g_status_list, 10, 65, L"登录码");
    add_column(g_status_list, 11, 75, L"Servo错误");
    add_column(g_status_list, 12, 70, L"速率Hz");
    add_column(g_status_list, 13, 55, L"丢包");
    add_column(g_status_list, 14, 75, L"Watchdog");
    add_column(g_status_list, 15, 80, L"读/写错误");

    HWND status_title = add_control(window, L"STATIC", L"机器人实时状态", SS_LEFT,
                                   24, 16, 520, 32, 0, nullptr);
    SendMessageW(status_title, WM_SETFONT, reinterpret_cast<WPARAM>(g_heading_font), TRUE);
    g_status_page_summary = add_control(window, L"STATIC", L"", SS_LEFT | SS_CENTERIMAGE,
                                        540, 20, 550, 40, ID_STATUS_PAGE_SUMMARY, nullptr);
    SendMessageW(g_status_page_summary, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);
    HWND back = add_control(window, L"BUTTON", L"返回控制台", WS_TABSTOP | BS_PUSHBUTTON,
                            1110, 20, 140, 40, ID_STATUS_BACK, nullptr);
    HWND primary_label = add_control(window, L"STATIC", L"运行状态", SS_LEFT,
                                    24, 74, 180, 24, 0, nullptr);
    SendMessageW(primary_label, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);
    g_status_page_primary = CreateWindowExW(WS_EX_CLIENTEDGE, WC_LISTVIEWW, L"",
                                            WS_CHILD | LVS_REPORT | LVS_SINGLESEL,
                                            24, 104, 1250, 360, window,
                                            reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_STATUS_PAGE_PRIMARY)),
                                            GetModuleHandleW(nullptr), nullptr);
    SendMessageW(g_status_page_primary, WM_SETFONT, reinterpret_cast<WPARAM>(g_small_font), TRUE);
    ListView_SetExtendedListViewStyle(g_status_page_primary,
                                      LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES | LVS_EX_DOUBLEBUFFER | LVS_EX_LABELTIP);
    add_column(g_status_page_primary, 0, 130, L"机器人");
    add_column(g_status_page_primary, 1, 110, L"模式");
    add_column(g_status_page_primary, 2, 80, L"连接");
    add_column(g_status_page_primary, 3, 80, L"上电");
    add_column(g_status_page_primary, 4, 80, L"使能");
    add_column(g_status_page_primary, 5, 80, L"拖动");
    add_column(g_status_page_primary, 6, 80, L"伺服");
    add_column(g_status_page_primary, 7, 360, L"故障");
    add_column(g_status_page_primary, 8, 120, L"数据年龄ms");
    add_column(g_status_page_primary, 9, 120, L"序列号");

    HWND diag_label = add_control(window, L"STATIC", L"通信与诊断", SS_LEFT,
                                  24, 480, 180, 24, 0, nullptr);
    SendMessageW(diag_label, WM_SETFONT, reinterpret_cast<WPARAM>(g_body_font), TRUE);
    g_status_page_diag = CreateWindowExW(WS_EX_CLIENTEDGE, WC_LISTVIEWW, L"",
                                         WS_CHILD | LVS_REPORT | LVS_SINGLESEL,
                                         24, 510, 1250, 440, window,
                                         reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_STATUS_PAGE_DIAG)),
                                         GetModuleHandleW(nullptr), nullptr);
    SendMessageW(g_status_page_diag, WM_SETFONT, reinterpret_cast<WPARAM>(g_small_font), TRUE);
    ListView_SetExtendedListViewStyle(g_status_page_diag,
                                      LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES | LVS_EX_DOUBLEBUFFER | LVS_EX_LABELTIP);
    add_column(g_status_page_diag, 0, 130, L"机器人");
    add_column(g_status_page_diag, 1, 110, L"登录返回码");
    add_column(g_status_page_diag, 2, 120, L"Servo错误码");
    add_column(g_status_page_diag, 3, 120, L"数据率Hz");
    add_column(g_status_page_diag, 4, 100, L"丢包数");
    add_column(g_status_page_diag, 5, 120, L"Watchdog");
    add_column(g_status_page_diag, 6, 150, L"读取/发送错误");
    add_column(g_status_page_diag, 7, 390, L"停止/故障原因");

    for (HWND control : {status_title, primary_label, diag_label, g_status_page_summary,
                                back, g_status_page_primary, g_status_page_diag}) {
        g_status_page_controls.push_back(control);
    }
    for (HWND control : g_status_page_controls) ShowWindow(control, SW_HIDE);

    fill_robot_list();
    fill_group_list();
    refresh_status_list();
    set_status(g_registry_path.empty() ? L"未找到配置文件" : L"配置：" + g_registry_path.wstring());
}

void clear_list(HWND list) {
    ListView_DeleteAllItems(list);
}

void fill_robot_list() {
    g_suppress_selection = true;
    clear_list(g_robot_list);
    for (std::size_t i = 0; i < g_registry.robots.size(); ++i) {
        const auto& robot = g_registry.robots[i];
        LVITEMW item{};
        item.mask = LVIF_TEXT;
        item.iItem = static_cast<int>(i);
        std::wstring id = widen(robot.id);
        item.pszText = id.data();
        const int row = ListView_InsertItem(g_robot_list, &item);
        std::wstring name = widen(robot.name);
        std::wstring ip = widen(robot.ip);
        ListView_SetItemText(g_robot_list, row, 1, name.data());
        ListView_SetItemText(g_robot_list, row, 2, ip.data());
    }
    g_suppress_selection = false;
}

void fill_group_list() {
    g_suppress_selection = true;
    clear_list(g_group_list);
    for (std::size_t i = 0; i < g_registry.groups.size(); ++i) {
        const auto& group = g_registry.groups[i];
        LVITEMW item{};
        item.mask = LVIF_TEXT;
        item.iItem = static_cast<int>(i);
        std::wstring id = widen(group.id);
        item.pszText = id.data();
        const int row = ListView_InsertItem(g_group_list, &item);
        std::wstring name = widen(group.name);
        ListView_SetItemText(g_group_list, row, 1, name.data());
    }
    g_suppress_selection = false;
}

int selected_index(HWND list) {
    return ListView_GetNextItem(list, -1, LVNI_SELECTED);
}

void set_combo_text(HWND combo, const std::string& value) {
    const std::wstring wide = widen(value);
    const int index = static_cast<int>(SendMessageW(combo, CB_FINDSTRINGEXACT, static_cast<WPARAM>(-1),
                                                    reinterpret_cast<LPARAM>(wide.c_str())));
    SendMessageW(combo, CB_SETCURSEL, index >= 0 ? index : 1, 0);
}

std::string combo_text(HWND combo) {
    const int index = static_cast<int>(SendMessageW(combo, CB_GETCURSEL, 0, 0));
    if (index < 0) return "none";
    wchar_t text[32]{};
    SendMessageW(combo, CB_GETLBTEXT, index, reinterpret_cast<LPARAM>(text));
    return narrow(text);
}

void load_robot_form(int index) {
    if (index < 0 || index >= static_cast<int>(g_registry.robots.size())) return;
    const auto& robot = g_registry.robots[static_cast<std::size_t>(index)];
    set_text(GetDlgItem(g_window, ID_ROBOT_ID), robot.id);
    set_text(GetDlgItem(g_window, ID_ROBOT_NAME), robot.name);
    set_text(GetDlgItem(g_window, ID_ROBOT_MODEL), robot.model);
    set_text(GetDlgItem(g_window, ID_ROBOT_IP), robot.ip);
    SendMessageW(GetDlgItem(g_window, ID_ROBOT_ENABLED), BM_SETCHECK,
                 robot.enabled ? BST_CHECKED : BST_UNCHECKED, 0);
    set_combo_text(GetDlgItem(g_window, ID_ROBOT_FILTER), robot.filter);
    set_text(GetDlgItem(g_window, ID_ROBOT_LPF), std::to_string(robot.lpf_cutoff));
    set_text(GetDlgItem(g_window, ID_ROBOT_VEL), std::to_string(robot.max_velocity));
    set_text(GetDlgItem(g_window, ID_ROBOT_ACC), std::to_string(robot.max_acceleration));
    set_text(GetDlgItem(g_window, ID_ROBOT_DIRECTION), format_numbers(robot.direction));
    set_text(GetDlgItem(g_window, ID_ROBOT_LOWER), format_numbers(robot.lower_rad, windows_jaka::kRadiansToDegrees));
    set_text(GetDlgItem(g_window, ID_ROBOT_UPPER), format_numbers(robot.upper_rad, windows_jaka::kRadiansToDegrees));
    set_text(GetDlgItem(g_window, ID_ROBOT_SAFE), format_numbers(robot.safe_pose_rad, windows_jaka::kRadiansToDegrees));
}

bool read_robot_form(windows_jaka::RobotProfile& robot, std::string& error) {
    robot = {};
    robot.id = read_text(GetDlgItem(g_window, ID_ROBOT_ID));
    robot.name = read_text(GetDlgItem(g_window, ID_ROBOT_NAME));
    robot.model = read_text(GetDlgItem(g_window, ID_ROBOT_MODEL));
    robot.ip = read_text(GetDlgItem(g_window, ID_ROBOT_IP));
    robot.enabled = SendMessageW(GetDlgItem(g_window, ID_ROBOT_ENABLED), BM_GETCHECK, 0, 0) == BST_CHECKED;
    robot.filter = combo_text(GetDlgItem(g_window, ID_ROBOT_FILTER));
    try {
        robot.lpf_cutoff = std::stod(read_text(GetDlgItem(g_window, ID_ROBOT_LPF)));
        robot.max_velocity = std::stod(read_text(GetDlgItem(g_window, ID_ROBOT_VEL)));
        robot.max_acceleration = std::stod(read_text(GetDlgItem(g_window, ID_ROBOT_ACC)));
    } catch (...) {
        error = "数值参数格式错误";
        return false;
    }
    if (!parse_numbers(read_text(GetDlgItem(g_window, ID_ROBOT_DIRECTION)), robot.direction) ||
        !parse_numbers(read_text(GetDlgItem(g_window, ID_ROBOT_LOWER)), robot.lower_rad,
                       windows_jaka::kDegreesToRadians) ||
        !parse_numbers(read_text(GetDlgItem(g_window, ID_ROBOT_UPPER)), robot.upper_rad,
                       windows_jaka::kDegreesToRadians) ||
        !parse_numbers(read_text(GetDlgItem(g_window, ID_ROBOT_SAFE)), robot.safe_pose_rad,
                       windows_jaka::kDegreesToRadians)) {
        error = "六关节参数必须是 6 个逗号分隔数值";
        return false;
    }
    return true;
}

std::unordered_map<std::string, std::string> read_status_file(const std::filesystem::path& path);
std::wstring status_yes_no(const std::unordered_map<std::string, std::string>& values,
                           const std::string& key);

bool is_status_page_control(HWND control) {
    return std::find(g_status_page_controls.begin(), g_status_page_controls.end(), control) !=
           g_status_page_controls.end();
}

void show_main_page() {
    g_status_page_open = false;
    for (HWND control = GetWindow(g_window, GW_CHILD); control;
         control = GetWindow(control, GW_HWNDNEXT)) {
        ShowWindow(control, is_status_page_control(control) ? SW_HIDE : SW_SHOW);
    }
    SetWindowTextW(g_window, L"JAKA 多机器人控制台");
    refresh_status_list();
    update_session_ui();
}

void show_status_page() {
    g_status_page_open = true;
    for (HWND control = GetWindow(g_window, GW_CHILD); control;
         control = GetWindow(control, GW_HWNDNEXT)) {
        ShowWindow(control, is_status_page_control(control) ? SW_SHOW : SW_HIDE);
    }
    SetWindowTextW(g_window, L"JAKA 多机器人控制台 - 实时状态");
    refresh_status_page();
}

void refresh_status_page() {
    if (!g_status_page_primary || !g_status_page_diag) return;
    clear_list(g_status_page_primary);
    clear_list(g_status_page_diag);
    int fault_count = 0;
    for (std::size_t i = 0; i < g_registry.robots.size(); ++i) {
        const auto& robot = g_registry.robots[i];
        const auto path = g_status_directory / (widen(robot.id) + L".status");
        const auto values = read_status_file(path);
        auto value = [&](const std::string& key, const std::wstring& fallback = L"-") -> std::wstring {
            const auto found = values.find(key);
            return found == values.end() || found->second.empty() ? fallback : widen(found->second);
        };
        std::wstring mode = values.empty() ? L"未运行" : value("mode", L"-");
        std::wstring alarm = value("alarm", L"");
        if (alarm.empty()) alarm = values.empty() ? L"未运行" : L"无";
        if (!values.empty() && alarm != L"无") ++fault_count;

        std::wstring id = widen(robot.id);
        LVITEMW item{};
        item.mask = LVIF_TEXT;
        item.iItem = static_cast<int>(i);
        item.pszText = id.data();
        const int row = ListView_InsertItem(g_status_page_primary, &item);
        std::wstring connected = values.empty() ? L"-" : status_yes_no(values, "connected");
        std::wstring powered = values.empty() ? L"-" : status_yes_no(values, "powered");
        std::wstring enabled = values.empty() ? L"-" : status_yes_no(values, "enabled");
        std::wstring dragging = values.empty() ? L"-" : status_yes_no(values, "dragging");
        std::wstring servo = values.empty() ? L"-" : status_yes_no(values, "servo");
        std::wstring age = value("packet_age_ms", L"-");
        std::wstring sequence = value("sequence", L"-");
        ListView_SetItemText(g_status_page_primary, row, 1, mode.data());
        ListView_SetItemText(g_status_page_primary, row, 2, connected.data());
        ListView_SetItemText(g_status_page_primary, row, 3, powered.data());
        ListView_SetItemText(g_status_page_primary, row, 4, enabled.data());
        ListView_SetItemText(g_status_page_primary, row, 5, dragging.data());
        ListView_SetItemText(g_status_page_primary, row, 6, servo.data());
        ListView_SetItemText(g_status_page_primary, row, 7, alarm.data());
        ListView_SetItemText(g_status_page_primary, row, 8, age.data());
        ListView_SetItemText(g_status_page_primary, row, 9, sequence.data());

        const int diag_row = ListView_InsertItem(g_status_page_diag, &item);
        std::wstring login_code = value("login_code", L"-");
        std::wstring servo_error = value("servo_error", L"-");
        std::wstring rate = value("rate_hz", L"-");
        std::wstring dropped = value("dropped_packets", L"-");
        std::wstring watchdog = value("watchdog_ticks", L"-");
        const std::wstring read_errors = values.empty() ? L"-" : value("read_errors", L"0");
        const std::wstring send_errors = values.empty() ? L"-" : value("send_errors", L"0");
        std::wstring errors = values.empty() ? L"-" : read_errors + L" / " + send_errors;
        ListView_SetItemText(g_status_page_diag, diag_row, 1, login_code.data());
        ListView_SetItemText(g_status_page_diag, diag_row, 2, servo_error.data());
        ListView_SetItemText(g_status_page_diag, diag_row, 3, rate.data());
        ListView_SetItemText(g_status_page_diag, diag_row, 4, dropped.data());
        ListView_SetItemText(g_status_page_diag, diag_row, 5, watchdog.data());
        ListView_SetItemText(g_status_page_diag, diag_row, 6, errors.data());
        ListView_SetItemText(g_status_page_diag, diag_row, 7, alarm.data());
    }

    SYSTEMTIME now{};
    GetLocalTime(&now);
    wchar_t summary[256]{};
    swprintf_s(summary, L"机器人 %zu 台  |  运行会话 %zu  |  故障 %d  |  更新 %02d:%02d:%02d",
               g_registry.robots.size(), g_sessions.size(), fault_count,
               now.wHour, now.wMinute, now.wSecond);
    SetWindowTextW(g_status_page_summary, summary);
}

std::unordered_map<std::string, std::string> read_status_file(const std::filesystem::path& path) {
    std::unordered_map<std::string, std::string> values;
    std::ifstream input(path);
    if (!input.is_open()) return values;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto equals = line.find('=');
        if (equals == std::string::npos) continue;
        values[line.substr(0, equals)] = line.substr(equals + 1);
    }
    return values;
}

std::wstring status_yes_no(const std::unordered_map<std::string, std::string>& values,
                           const std::string& key) {
    const auto found = values.find(key);
    if (found == values.end()) return L"-";
    return (found->second == "1" || found->second == "true") ? L"是" : L"否";
}

void refresh_status_list() {
    if (!g_status_list) return;
    g_suppress_selection = true;
    ListView_DeleteAllItems(g_status_list);
    for (std::size_t i = 0; i < g_registry.robots.size(); ++i) {
        const auto& robot = g_registry.robots[i];
        const auto path = g_status_directory / (widen(robot.id) + L".status");
        const auto values = read_status_file(path);
        auto value = [&](const std::string& key, const std::wstring& fallback = L"-") -> std::wstring {
            const auto found = values.find(key);
            return found == values.end() || found->second.empty() ? fallback : widen(found->second);
        };
        std::wstring mode = value("mode", L"未运行");
        if (values.empty()) mode = L"未运行";
        std::wstring alarm = value("alarm", L"");
        if (alarm.empty()) alarm = L"无";

        std::wstring id = widen(robot.id);
        LVITEMW item{};
        item.mask = LVIF_TEXT;
        item.iItem = static_cast<int>(i);
        item.pszText = id.data();
        const int row = ListView_InsertItem(g_status_list, &item);
        std::wstring connected = values.empty() ? L"-" : status_yes_no(values, "connected");
        std::wstring powered = values.empty() ? L"-" : status_yes_no(values, "powered");
        std::wstring enabled = values.empty() ? L"-" : status_yes_no(values, "enabled");
        std::wstring dragging = values.empty() ? L"-" : status_yes_no(values, "dragging");
        std::wstring servo = values.empty() ? L"-" : status_yes_no(values, "servo");
        std::wstring age = value("packet_age_ms", L"-");
        std::wstring sequence = value("sequence", L"-");
        ListView_SetItemText(g_status_list, row, 1, mode.data());
        ListView_SetItemText(g_status_list, row, 2, connected.data());
        ListView_SetItemText(g_status_list, row, 3, powered.data());
        ListView_SetItemText(g_status_list, row, 4, enabled.data());
        ListView_SetItemText(g_status_list, row, 5, dragging.data());
        ListView_SetItemText(g_status_list, row, 6, servo.data());
        ListView_SetItemText(g_status_list, row, 7, alarm.data());
        ListView_SetItemText(g_status_list, row, 8, age.data());
        ListView_SetItemText(g_status_list, row, 9, sequence.data());
        std::wstring login_code = value("login_code", L"");
        std::wstring servo_error = value("servo_error", L"");
        std::wstring rate = value("rate_hz", L"");
        std::wstring dropped = value("dropped_packets", L"");
        std::wstring watchdog = value("watchdog_ticks", L"");
        const std::wstring read_errors = values.empty() ? L"-" : value("read_errors", L"0");
        const std::wstring send_errors = values.empty() ? L"-" : value("send_errors", L"0");
        std::wstring errors = values.empty() ? L"-" : read_errors + L"/" + send_errors;
        ListView_SetItemText(g_status_list, row, 10, login_code.data());
        ListView_SetItemText(g_status_list, row, 11, servo_error.data());
        ListView_SetItemText(g_status_list, row, 12, rate.data());
        ListView_SetItemText(g_status_list, row, 13, dropped.data());
        ListView_SetItemText(g_status_list, row, 14, watchdog.data());
        ListView_SetItemText(g_status_list, row, 15, errors.data());
    }
    g_suppress_selection = false;
}

void load_group_form(int index) {
    if (index < 0 || index >= static_cast<int>(g_registry.groups.size())) return;
    const auto& group = g_registry.groups[static_cast<std::size_t>(index)];
    set_text(GetDlgItem(g_window, ID_GROUP_ID), group.id);
    set_text(GetDlgItem(g_window, ID_GROUP_NAME), group.name);
    set_text(GetDlgItem(g_window, ID_GROUP_OPERATOR), group.operator_robot_id);
    std::ostringstream followers;
    for (std::size_t i = 0; i < group.follower_robot_ids.size(); ++i) {
        if (i != 0) followers << ',';
        followers << group.follower_robot_ids[i];
    }
    set_text(GetDlgItem(g_window, ID_GROUP_FOLLOWERS), followers.str());
}

bool read_group_form(windows_jaka::TeleopGroup& group, std::string& error) {
    group = {};
    group.id = read_text(GetDlgItem(g_window, ID_GROUP_ID));
    group.name = read_text(GetDlgItem(g_window, ID_GROUP_NAME));
    group.operator_robot_id = read_text(GetDlgItem(g_window, ID_GROUP_OPERATOR));
    std::string followers = read_text(GetDlgItem(g_window, ID_GROUP_FOLLOWERS));
    std::stringstream input(followers);
    std::string item;
    while (std::getline(input, item, ',')) {
        const auto first = item.find_first_not_of(" \t");
        const auto last = item.find_last_not_of(" \t");
        if (first == std::string::npos) continue;
        group.follower_robot_ids.push_back(item.substr(first, last - first + 1));
    }
    if (group.id.empty() || group.name.empty() || group.operator_robot_id.empty() ||
        group.follower_robot_ids.empty()) {
        error = "组 ID、名称、操作臂和跟随臂不能为空";
        return false;
    }
    return true;
}

void select_list_item(HWND list, int index) {
    ListView_SetItemState(list, -1, 0, LVIS_SELECTED | LVIS_FOCUSED);
    if (index >= 0) {
        ListView_SetItemState(list, index, LVIS_SELECTED | LVIS_FOCUSED,
                              LVIS_SELECTED | LVIS_FOCUSED);
        ListView_EnsureVisible(list, index, FALSE);
    }
}

bool save_registry() {
    std::error_code directory_error;
    std::filesystem::create_directories(g_registry_path.parent_path(), directory_error);
    if (directory_error) {
        set_status(L"无法创建配置目录：" + widen(directory_error.message()));
        return false;
    }
    std::string error;
    if (!g_registry.save(g_registry_path, error)) {
        set_status(L"保存失败：" + widen(error));
        return false;
    }
    set_status(L"已保存：" + g_registry_path.wstring());
    return true;
}

std::wstring quote_w(const std::wstring& value) {
    return L"\"" + value + L"\"";
}

std::filesystem::path group_launcher_path() {
    return workdir_root() / L"windows_jaka" / L"run_group_from_registry.ps1";
}

std::filesystem::path single_launcher_path() {
    return workdir_root() / L"windows_jaka" / L"run_single_robot.ps1";
}

std::string group_session_key(const std::string& id) { return "group:" + id; }
std::string robot_session_key(const std::string& id) { return "robot:" + id; }

ActiveSession* find_session(const std::string& key) {
    for (auto& session : g_sessions) {
        if (session->key == key) return session.get();
    }
    return nullptr;
}

bool any_session_running() {
    for (const auto& session : g_sessions) {
        if (session->process && WaitForSingleObject(session->process, 0) == WAIT_TIMEOUT) return true;
    }
    return false;
}

bool session_running() { return any_session_running(); }

bool robot_in_use(const std::string& robot_id, const std::string& except_key = {}) {
    for (const auto& session : g_sessions) {
        if (!except_key.empty() && session->key == except_key) continue;
        if (std::find(session->robot_ids.begin(), session->robot_ids.end(), robot_id) !=
            session->robot_ids.end()) return true;
    }
    return false;
}

std::uint16_t allocate_group_port(int follower_count) {
    int candidate = 30101;
    while (true) {
        bool conflict = false;
        for (const auto& session : g_sessions) {
            if (session->kind != SessionKind::Group) continue;
            const int existing_start = session->base_port;
            const int existing_end = existing_start + static_cast<int>(session->robot_ids.size()) - 1;
            const int candidate_end = candidate + follower_count - 1;
            if (!(candidate_end < existing_start || candidate > existing_end)) {
                conflict = true;
                break;
            }
        }
        if (!conflict) {
            if (candidate + follower_count > 65535) throw std::runtime_error("no UDP ports available");
            return static_cast<std::uint16_t>(candidate);
        }
        candidate += 100;
    }
}

bool launch_session_command(const std::wstring& command, bool real_motion,
                            const std::filesystem::path& log_path,
                            HANDLE& process_out, HANDLE& job_out, DWORD& error_code) {
    std::vector<wchar_t> command_line(command.begin(), command.end());
    command_line.push_back(L'\0');
    std::wstring old_motion;
    const DWORD old_length = GetEnvironmentVariableW(L"JAKA_ENABLE_MOTION", nullptr, 0);
    if (old_length > 0) {
        old_motion.resize(old_length);
        GetEnvironmentVariableW(L"JAKA_ENABLE_MOTION", old_motion.data(), old_length);
        old_motion.resize(old_length - 1);
    }
    SetEnvironmentVariableW(L"JAKA_ENABLE_MOTION", real_motion ? L"1" : nullptr);

    std::error_code directory_error;
    std::filesystem::create_directories(log_path.parent_path(), directory_error);
    SECURITY_ATTRIBUTES security{};
    security.nLength = sizeof(security);
    security.bInheritHandle = TRUE;
    HANDLE log_handle = CreateFileW(log_path.c_str(), FILE_APPEND_DATA,
                                    FILE_SHARE_READ | FILE_SHARE_WRITE, &security,
                                    OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (log_handle != INVALID_HANDLE_VALUE) {
        SetFilePointer(log_handle, 0, nullptr, FILE_END);
    }

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    if (log_handle != INVALID_HANDLE_VALUE) {
        startup.dwFlags = STARTF_USESTDHANDLES;
        startup.hStdOutput = log_handle;
        startup.hStdError = log_handle;
        startup.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    }
    PROCESS_INFORMATION process{};
    const BOOL created = CreateProcessW(nullptr, command_line.data(), nullptr, nullptr,
                                        log_handle != INVALID_HANDLE_VALUE ? TRUE : FALSE,
                                        CREATE_NO_WINDOW | CREATE_NEW_PROCESS_GROUP,
                                        nullptr, workdir_root().c_str(), &startup, &process);
    if (log_handle != INVALID_HANDLE_VALUE) CloseHandle(log_handle);
    error_code = created ? ERROR_SUCCESS : GetLastError();
    SetEnvironmentVariableW(L"JAKA_ENABLE_MOTION", old_motion.empty() ? nullptr : old_motion.c_str());
    if (!created) return false;

    CloseHandle(process.hThread);
    process_out = process.hProcess;
    job_out = CreateJobObjectW(nullptr, nullptr);
    if (job_out) {
        JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
        limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
        if (!SetInformationJobObject(job_out, JobObjectExtendedLimitInformation,
                                     &limits, sizeof(limits)) ||
            !AssignProcessToJobObject(job_out, process_out)) {
            CloseHandle(job_out);
            job_out = nullptr;
        }
    }
    return true;
}

void terminate_session(ActiveSession* session, bool graceful) {
    if (!session) return;
    if (graceful) {
        for (const auto& pipe : session->stop_pipes) {
            windows_jaka::send_control_command(pipe, "STOP");
        }
    }
    if (session->process && WaitForSingleObject(session->process, graceful ? 3000 : 0) == WAIT_TIMEOUT) {
        if (session->job) TerminateJobObject(session->job, 1);
        else TerminateProcess(session->process, 1);
        WaitForSingleObject(session->process, 1000);
    }
}

void remove_session(const std::string& key) {
    g_sessions.erase(std::remove_if(g_sessions.begin(), g_sessions.end(),
        [&](const std::unique_ptr<ActiveSession>& session) { return session->key == key; }),
        g_sessions.end());
}

std::wstring last_log_line(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::string line;
    std::string last;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) last = line;
    }
    return widen(last);
}

void prune_exited_sessions() {
    bool exited = false;
    std::wstring message;
    for (const auto& session : g_sessions) {
        if (session->process && WaitForSingleObject(session->process, 0) != WAIT_TIMEOUT) {
            exited = true;
            const std::wstring detail = last_log_line(session->log_path);
            message = L"会话已退出：" + widen(session->label);
            if (!detail.empty()) message += L" - " + detail;
            break;
        }
    }
    g_sessions.erase(std::remove_if(g_sessions.begin(), g_sessions.end(),
        [](const std::unique_ptr<ActiveSession>& session) {
            return !session->process || WaitForSingleObject(session->process, 0) != WAIT_TIMEOUT;
        }), g_sessions.end());
    if (exited) {
        set_status(message);
        if (g_session_status && g_sessions.empty()) SetWindowTextW(g_session_status, message.c_str());
    }
}

void update_session_ui() {
    prune_exited_sessions();
    const int group_index = selected_index(g_group_list);
    const int robot_index = selected_index(g_robot_list);
    std::string group_id;
    std::string robot_id;
    if (group_index >= 0 && group_index < static_cast<int>(g_registry.groups.size())) {
        group_id = g_registry.groups[static_cast<std::size_t>(group_index)].id;
    }
    if (robot_index >= 0 && robot_index < static_cast<int>(g_registry.robots.size())) {
        robot_id = g_registry.robots[static_cast<std::size_t>(robot_index)].id;
    }
    ActiveSession* group_session = group_id.empty() ? nullptr : find_session(group_session_key(group_id));
    ActiveSession* robot_session = robot_id.empty() ? nullptr : find_session(robot_session_key(robot_id));
    const bool single_joint = robot_session && robot_session->kind == SessionKind::Single &&
        (robot_session->mode == "joint" || robot_session->mode == "record");

    EnableWindow(GetDlgItem(g_window, ID_GROUP_START), (!group_id.empty() && !group_session) ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_window, ID_GROUP_STOP), group_session ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_window, ID_SINGLE_JOINT),
                 (!robot_id.empty() && !robot_session && !robot_in_use(robot_id)) ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_window, ID_SINGLE_RECORD),
                 (!robot_id.empty() && !robot_session && !robot_in_use(robot_id)) ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_window, ID_SINGLE_PLAYBACK),
                 (!robot_id.empty() && !robot_session && !robot_in_use(robot_id)) ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_window, ID_SINGLE_STOP), robot_session ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_window, ID_STOP_ALL), g_sessions.empty() ? FALSE : TRUE);
    for (int axis = 0; axis < 6; ++axis) {
        EnableWindow(GetDlgItem(g_window, ID_JOG_BASE + axis * 2), single_joint ? TRUE : FALSE);
        EnableWindow(GetDlgItem(g_window, ID_JOG_BASE + axis * 2 + 1), single_joint ? TRUE : FALSE);
    }
    EnableWindow(GetDlgItem(g_window, ID_SAFE_EXECUTE), single_joint ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_window, ID_GROUP_REAL_MOTION), TRUE);

    if (g_session_status) {
        if (g_sessions.empty()) SetWindowTextW(g_session_status, L"无运行会话");
        else SetWindowTextW(g_session_status, (L"运行中会话：" + std::to_wstring(g_sessions.size())).c_str());
    }
}

void start_selected_group() {
    const int index = selected_index(g_group_list);
    if (index < 0 || index >= static_cast<int>(g_registry.groups.size())) {
        set_status(L"请先选择一个遥操作组");
        return;
    }
    if (!save_registry()) return;
    const std::string group_id = g_registry.groups[static_cast<std::size_t>(index)].id;
    if (find_session(group_session_key(group_id))) {
        set_status(L"该组已经在运行");
        return;
    }
    windows_jaka::TeleopRuntimePlan plan;
    std::string error;
    if (!windows_jaka::build_teleop_runtime_plan(g_registry, group_id, 30101, plan, error)) {
        set_status(L"运行计划无效：" + widen(error));
        return;
    }
    for (const auto& robot : plan.follower_robots) {
        if (robot_in_use(robot.id)) {
            set_status(L"机器人已被其他会话占用：" + widen(robot.id));
            return;
        }
    }
    if (robot_in_use(plan.operator_robot.id)) {
        set_status(L"操作臂已被其他会话占用：" + widen(plan.operator_robot.id));
        return;
    }
    const auto launcher = group_launcher_path();
    if (!std::filesystem::exists(launcher)) {
        set_status(L"找不到组启动脚本：" + launcher.wstring());
        return;
    }
    std::uint16_t base_port = 0;
    try {
        base_port = allocate_group_port(static_cast<int>(plan.follower_endpoints.size()));
    } catch (const std::exception& exception) {
        set_status(widen(exception.what()));
        return;
    }
    const bool real_motion =
        SendMessageW(GetDlgItem(g_window, ID_GROUP_REAL_MOTION), BM_GETCHECK, 0, 0) == BST_CHECKED;
    std::wstring command = L"powershell.exe -NoProfile -ExecutionPolicy Bypass -File " +
        quote_w(launcher.wstring()) +
        L" -RegistryPath " + quote_w(g_registry_path.wstring()) +
        L" -GroupId " + quote_w(widen(group_id)) +
        L" -BasePort " + std::to_wstring(base_port) +
        L" -StatusDirectory " + quote_w(g_status_directory.wstring());
    command += real_motion ? L" -ArmMotion" : L" -DryRun";

    std::error_code ignored;
    std::filesystem::remove(g_status_directory / (widen(plan.operator_robot.id) + L".status"), ignored);
    for (const auto& follower : plan.follower_robots) {
        std::filesystem::remove(g_status_directory / (widen(follower.id) + L".status"), ignored);
    }
    HANDLE process = nullptr;
    HANDLE job = nullptr;
    DWORD error_code = ERROR_SUCCESS;
    if (!launch_session_command(command, real_motion, g_status_directory / (widen(group_id) + L".log"), process, job, error_code)) {
        set_status(L"启动组失败，Windows 错误码=" + std::to_wstring(error_code));
        return;
    }
    auto session = std::make_unique<ActiveSession>();
    session->key = group_session_key(group_id);
    session->label = group_id;
    session->kind = SessionKind::Group;
    session->process = process;
    session->job = job;
    session->control_pipe = widen(plan.operator_control_pipe);
    session->stop_pipes.push_back(session->control_pipe);
    session->robot_ids.push_back(plan.operator_robot.id);
    for (const auto& endpoint : plan.follower_endpoints) {
        session->stop_pipes.push_back(widen(endpoint.control_pipe));
        session->robot_ids.push_back(endpoint.robot_id);
    }
    session->base_port = base_port;
    session->log_path = g_status_directory / (widen(group_id) + L".log");
    g_sessions.push_back(std::move(session));
    set_status((real_motion ? L"真实运动组已启动：" : L"Dry-run 组已启动：") +
               widen(group_id) + L"，跟随臂=" + std::to_wstring(plan.follower_endpoints.size()));
    update_session_ui();
}

bool send_single_pipe_line(const std::string& line) {
    const int index = selected_index(g_robot_list);
    if (index < 0 || index >= static_cast<int>(g_registry.robots.size())) return false;
    const std::string robot_id = g_registry.robots[static_cast<std::size_t>(index)].id;
    ActiveSession* session = find_session(robot_session_key(robot_id));
    if (!session || session->kind != SessionKind::Single) {
        set_status(L"当前没有运行中的单台会话");
        return false;
    }
    if (!windows_jaka::send_control_command(session->control_pipe, line)) {
        set_status(L"单台控制管道连接失败");
        return false;
    }
    return true;
}

void execute_single_safe_pose() {
    const int index = selected_index(g_robot_list);
    if (index < 0 || index >= static_cast<int>(g_registry.robots.size())) {
        set_status(L"请先选择机器人");
        return;
    }
    windows_jaka::JointArray pose{};
    if (!parse_numbers(read_text(GetDlgItem(g_window, ID_ROBOT_SAFE)), pose,
                       windows_jaka::kDegreesToRadians)) {
        set_status(L"安全姿态必须是 6 个逗号分隔角度值");
        return;
    }
    const auto& robot = g_registry.robots[static_cast<std::size_t>(index)];
    for (int i = 0; i < 6; ++i) {
        if (pose[static_cast<std::size_t>(i)] < robot.lower_rad[static_cast<std::size_t>(i)] ||
            pose[static_cast<std::size_t>(i)] > robot.upper_rad[static_cast<std::size_t>(i)]) {
            set_status(L"安全姿态超出该机器人关节限位");
            return;
        }
    }
    std::ostringstream command;
    command << "SAFEPOSE";
    for (double value : pose) command << ' ' << value;
    if (send_single_pipe_line(command.str())) set_status(L"安全姿态命令已发送");
}

LRESULT CALLBACK jog_button_subclass(HWND hwnd, UINT message, WPARAM wparam,
                                     LPARAM lparam, UINT_PTR, DWORD_PTR data) {
    const int axis = static_cast<int>(data) / 2;
    const bool positive = (static_cast<int>(data) % 2) != 0;
    const double delta = (axis < 3 ? 0.002 : 0.0015) * (positive ? 1.0 : -1.0);
    if (message == WM_LBUTTONDOWN) {
        const int index = selected_index(g_robot_list);
        if (index < 0 || index >= static_cast<int>(g_registry.robots.size())) return 0;
        ActiveSession* session = find_session(robot_session_key(
            g_registry.robots[static_cast<std::size_t>(index)].id));
        if (!session || session->kind != SessionKind::Single ||
            !(session->mode == "joint" || session->mode == "record")) {
            set_status(L"请先启动单臂关节控制或轨迹录制");
            return 0;
        }
        g_jog_axis = axis;
        g_jog_delta = delta;
        SetCapture(hwnd);
        send_single_pipe_line("JOG " + std::to_string(axis) + " " + std::to_string(delta));
        return 0;
    }
    if (message == WM_LBUTTONUP || message == WM_CAPTURECHANGED) {
        if (g_jog_axis == axis) {
            send_single_pipe_line("JOG_STOP " + std::to_string(axis));
            g_jog_axis = -1;
            g_jog_delta = 0.0;
        }
        if (GetCapture() == hwnd) ReleaseCapture();
        return 0;
    }
    return DefSubclassProc(hwnd, message, wparam, lparam);
}

void start_single_session(const std::string& control_mode) {
    const int index = selected_index(g_robot_list);
    if (index < 0 || index >= static_cast<int>(g_registry.robots.size())) {
        set_status(L"请先选择一台机器人");
        return;
    }
    if (!save_registry()) return;
    const auto& robot = g_registry.robots[static_cast<std::size_t>(index)];
    if (!robot.enabled) {
        set_status(L"选中的机器人已被禁用");
        return;
    }
    if (robot_in_use(robot.id)) {
        set_status(L"机器人已被其他会话占用：" + widen(robot.id));
        return;
    }
    const auto launcher = single_launcher_path();
    if (!std::filesystem::exists(launcher)) {
        set_status(L"找不到单台启动脚本：" + launcher.wstring());
        return;
    }

    std::wstring playback_file;
    if (control_mode == "playback") {
        wchar_t file_name[MAX_PATH]{};
        OPENFILENAMEW dialog{};
        dialog.lStructSize = sizeof(dialog);
        dialog.hwndOwner = g_window;
        dialog.lpstrFilter = L"JAKA 关节轨迹 (*.csv)\0*.csv\0所有文件 (*.*)\0*.*\0\0";
        dialog.lpstrFile = file_name;
        dialog.nMaxFile = static_cast<DWORD>(std::size(file_name));
        const std::wstring initial_log_dir = (workdir_root() / L"logs").wstring();
        dialog.lpstrInitialDir = initial_log_dir.c_str();
        dialog.lpstrTitle = L"选择关节轨迹";
        dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_EXPLORER | OFN_NOCHANGEDIR;
        if (!GetOpenFileNameW(&dialog)) return;
        playback_file = file_name;
    }

    const bool real_motion =
        SendMessageW(GetDlgItem(g_window, ID_GROUP_REAL_MOTION), BM_GETCHECK, 0, 0) == BST_CHECKED;
    const std::wstring pipe = L"\\\\.\\pipe\\jaka_single_" +
        widen(windows_jaka::safe_pipe_component(robot.id));
    std::wstring command = L"powershell.exe -NoProfile -ExecutionPolicy Bypass -File " +
        quote_w(launcher.wstring()) +
        L" -RegistryPath " + quote_w(g_registry_path.wstring()) +
        L" -RobotId " + quote_w(widen(robot.id)) +
        L" -ControlMode " + quote_w(widen(control_mode)) +
        L" -StatusDirectory " + quote_w(g_status_directory.wstring());
    if (!playback_file.empty()) command += L" -PlaybackFile " + quote_w(playback_file);
    command += real_motion ? L" -ArmMotion" : L" -DryRun";

    std::error_code ignored;
    std::filesystem::remove(g_status_directory / (widen(robot.id) + L".status"), ignored);
    HANDLE process = nullptr;
    HANDLE job = nullptr;
    DWORD error_code = ERROR_SUCCESS;
    if (!launch_session_command(command, real_motion, g_status_directory / (widen(robot.id) + L".log"), process, job, error_code)) {
        set_status(L"启动单台机器人失败，Windows 错误码=" + std::to_wstring(error_code));
        return;
    }
    auto session = std::make_unique<ActiveSession>();
    session->key = robot_session_key(robot.id);
    session->label = robot.id;
    session->kind = SessionKind::Single;
    session->process = process;
    session->job = job;
    session->control_pipe = pipe;
    session->stop_pipes.push_back(pipe);
    session->robot_ids.push_back(robot.id);
    session->mode = control_mode;
    session->log_path = g_status_directory / (widen(robot.id) + L".log");
    g_sessions.push_back(std::move(session));
    set_status((real_motion ? L"真实运动单台已启动：" : L"Dry-run 单台已启动：") +
               widen(robot.id) + L"，模式=" + widen(control_mode));
    update_session_ui();
}

void stop_session_by_key(const std::string& key) {
    ActiveSession* session = find_session(key);
    if (!session) return;
    terminate_session(session, true);
    const std::wstring label = widen(session->label);
    remove_session(key);
    set_status(L"会话已停止：" + label);
    update_session_ui();
}

void stop_selected_group() {
    const int index = selected_index(g_group_list);
    if (index < 0 || index >= static_cast<int>(g_registry.groups.size())) return;
    stop_session_by_key(group_session_key(g_registry.groups[static_cast<std::size_t>(index)].id));
}

void stop_selected_single() {
    const int index = selected_index(g_robot_list);
    if (index < 0 || index >= static_cast<int>(g_registry.robots.size())) return;
    stop_session_by_key(robot_session_key(g_registry.robots[static_cast<std::size_t>(index)].id));
}

void stop_all_sessions() {
    std::vector<std::string> keys;
    keys.reserve(g_sessions.size());
    for (const auto& session : g_sessions) keys.push_back(session->key);
    for (const auto& key : keys) stop_session_by_key(key);
    set_status(L"全部会话已停止");
}

bool apply_selected_robot(bool show_error) {
    const int index = selected_index(g_robot_list);
    if (index < 0 || index >= static_cast<int>(g_registry.robots.size())) return false;
    windows_jaka::RobotProfile robot;
    std::string error;
    if (!read_robot_form(robot, error)) {
        set_status(L"机器人参数无效：" + widen(error));
        if (show_error) {
            MessageBoxW(g_window, widen(error).c_str(), L"机器人参数无效",
                        MB_ICONWARNING | MB_OK);
        }
        return false;
    }
    g_registry.robots[static_cast<std::size_t>(index)] = robot;
    fill_robot_list();
    select_list_item(g_robot_list, index);
    set_status(L"机器人已应用：" + widen(robot.id) + L"  IP=" + widen(robot.ip));
    return true;
}

bool apply_selected_group(bool show_error) {
    const int index = selected_index(g_group_list);
    if (index < 0 || index >= static_cast<int>(g_registry.groups.size())) return false;
    windows_jaka::TeleopGroup group;
    std::string error;
    if (!read_group_form(group, error)) {
        set_status(L"组参数无效：" + widen(error));
        if (show_error) {
            MessageBoxW(g_window, widen(error).c_str(), L"组参数无效",
                        MB_ICONWARNING | MB_OK);
        }
        return false;
    }
    g_registry.groups[static_cast<std::size_t>(index)] = group;
    fill_group_list();
    select_list_item(g_group_list, index);
    set_status(L"遥操作组已应用：" + widen(group.id));
    return true;
}

void handle_command(int id) {
    switch (id) {
    case ID_ROBOT_ADD: {
        g_registry.robots.push_back({});
        auto& robot = g_registry.robots.back();
        robot.id = "robot_" + std::to_string(g_registry.robots.size());
        robot.name = robot.id;
        robot.ip = "192.168.1.100";
        robot.safe_pose_rad = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        fill_robot_list();
        select_list_item(g_robot_list, static_cast<int>(g_registry.robots.size() - 1));
        load_robot_form(static_cast<int>(g_registry.robots.size() - 1));
        break;
    }
    case ID_ROBOT_APPLY:
        apply_selected_robot(true);
        break;
    case ID_ROBOT_DELETE: {
        const int index = selected_index(g_robot_list);
        if (index < 0) break;
        const std::string robot_id = g_registry.robots[static_cast<std::size_t>(index)].id;
        for (const auto& group : g_registry.groups) {
            if (group.operator_robot_id == robot_id ||
                std::find(group.follower_robot_ids.begin(), group.follower_robot_ids.end(), robot_id) !=
                    group.follower_robot_ids.end()) {
                set_status(L"机器人仍被遥操作组引用，不能删除");
                return;
            }
        }
        g_registry.robots.erase(g_registry.robots.begin() + index);
        fill_robot_list();
        break;
    }
    case ID_GROUP_ADD: {
        g_registry.groups.push_back({});
        auto& group = g_registry.groups.back();
        group.id = "group_" + std::to_string(g_registry.groups.size());
        group.name = group.id;
        fill_group_list();
        select_list_item(g_group_list, static_cast<int>(g_registry.groups.size() - 1));
        load_group_form(static_cast<int>(g_registry.groups.size() - 1));
        break;
    }
    case ID_GROUP_APPLY:
        apply_selected_group(true);
        break;
    case ID_GROUP_DELETE: {
        const int index = selected_index(g_group_list);
        if (index < 0) break;
        g_registry.groups.erase(g_registry.groups.begin() + index);
        fill_group_list();
        break;
    }
    case ID_SAVE: {
        bool ok = true;
        if (selected_index(g_robot_list) >= 0) ok = apply_selected_robot(true) && ok;
        if (selected_index(g_group_list) >= 0) ok = apply_selected_group(true) && ok;
        if (ok) save_registry();
        break;
    }
    case ID_SINGLE_JOINT:
        start_single_session("joint");
        break;
    case ID_SINGLE_RECORD:
        start_single_session("record");
        break;
    case ID_SINGLE_PLAYBACK:
        start_single_session("playback");
        break;
    case ID_SINGLE_STOP:
        stop_selected_single();
        break;
    case ID_STOP_ALL:
        stop_all_sessions();
        break;
    case ID_OPEN_STATUS_PAGE:
        show_status_page();
        break;
    case ID_STATUS_BACK:
        show_main_page();
        break;
    case ID_SAFE_EXECUTE:
        execute_single_safe_pose();
        break;
    case ID_GROUP_START:
        start_selected_group();
        break;
    case ID_GROUP_STOP:
        stop_selected_group();
        break;
    case ID_RELOAD: {
        std::string error;
        if (g_registry.load(g_registry_path, error)) {
            fill_robot_list();
            fill_group_list();
            set_status(L"已重新加载：" + g_registry_path.wstring());
        } else {
            set_status(L"加载失败：" + widen(error));
        }
        break;
    }
    default:
        break;
    }
}

LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    switch (message) {
    case WM_CREATE:
        g_window = window;
        build_ui(window);
        update_session_ui();
        SetTimer(window, ID_SESSION_TIMER, 500, nullptr);
        return 0;
    case WM_GETMINMAXINFO: {
        auto* limits = reinterpret_cast<MINMAXINFO*>(lparam);
        limits->ptMinTrackSize.x = 1280;
        limits->ptMinTrackSize.y = 1050;
        return 0;
    }
    case WM_ERASEBKGND: {
        RECT rect{};
        GetClientRect(window, &rect);
        FillRect(reinterpret_cast<HDC>(wparam), &rect, g_background_brush);
        RECT header{0, 0, rect.right, 82};
        FillRect(reinterpret_cast<HDC>(wparam), &header, g_surface_brush);
        return 1;
    }
    case WM_CTLCOLORSTATIC: {
        HDC dc = reinterpret_cast<HDC>(wparam);
        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, kText);
        return reinterpret_cast<LRESULT>(g_background_brush);
    }
    case WM_CTLCOLOREDIT:
    case WM_CTLCOLORLISTBOX: {
        HDC dc = reinterpret_cast<HDC>(wparam);
        SetBkColor(dc, kSurface);
        SetTextColor(dc, kText);
        return reinterpret_cast<LRESULT>(g_surface_brush);
    }
    case WM_CTLCOLORBTN: {
        HDC dc = reinterpret_cast<HDC>(wparam);
        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, kText);
        return reinterpret_cast<LRESULT>(g_background_brush);
    }
    case WM_PAINT: {
        PAINTSTRUCT paint{};
        HDC dc = BeginPaint(window, &paint);
        RECT client{};
        GetClientRect(window, &client);
        HPEN pen = CreatePen(PS_SOLID, 1, kBorder);
        HGDIOBJ old_pen = SelectObject(dc, pen);
        MoveToEx(dc, 0, 82, nullptr);
        LineTo(dc, client.right, 82);
        SelectObject(dc, old_pen);
        DeleteObject(pen);
        EndPaint(window, &paint);
        return 0;
    }
    case WM_TIMER:
        if (wparam == ID_SESSION_TIMER) {
            const std::size_t before = g_sessions.size();
            prune_exited_sessions();
            if (g_status_page_open) refresh_status_page();
            else refresh_status_list();
            update_session_ui();
            if (g_sessions.size() != before && g_sessions.empty() && g_session_status) {
                SetWindowTextW(g_session_status, L"全部会话已退出");
            }
        }
        return 0;
    case WM_COMMAND:
        if (HIWORD(wparam) == BN_CLICKED) handle_command(LOWORD(wparam));
        return 0;
    case WM_NOTIFY: {
        const auto* header = reinterpret_cast<const NMHDR*>(lparam);
        if (!header || g_suppress_selection) break;
        if (header->idFrom == ID_ROBOT_LIST && header->code == LVN_ITEMCHANGED) {
            const int index = selected_index(g_robot_list);
            if (index >= 0) load_robot_form(index);
        } else if (header->idFrom == ID_GROUP_LIST && header->code == LVN_ITEMCHANGED) {
            const int index = selected_index(g_group_list);
            if (index >= 0) load_group_form(index);
        }
        break;
    }
    case WM_DESTROY:
        stop_all_sessions();
        KillTimer(window, ID_SESSION_TIMER);
        DeleteObject(g_heading_font);
        DeleteObject(g_body_font);
        DeleteObject(g_small_font);
        DeleteObject(g_background_brush);
        DeleteObject(g_surface_brush);
        PostQuitMessage(0);
        return 0;
    default:
        break;
    }
    return DefWindowProcW(window, message, wparam, lparam);
}

std::filesystem::path workdir_root() {
    wchar_t module[MAX_PATH]{};
    GetModuleFileNameW(nullptr, module, MAX_PATH);
    const auto executable_dir = std::filesystem::path(module).parent_path();
    const auto workspace = executable_dir.parent_path().parent_path().parent_path();
    return std::filesystem::exists(workspace) ? workspace : executable_dir;
}

std::filesystem::path registry_path() {
    wchar_t module[MAX_PATH]{};
    GetModuleFileNameW(nullptr, module, MAX_PATH);
    const auto executable_dir = std::filesystem::path(module).parent_path();
    const auto local = executable_dir / L"config" / L"robots.ini";
    if (std::filesystem::exists(local)) return local;
    return executable_dir.parent_path().parent_path().parent_path() / L"config" / L"robots.ini";
}

std::filesystem::path example_registry_path() {
    return workdir_root() / L"config" / L"robots.example.ini";
}

}  // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show_command) {
    INITCOMMONCONTROLSEX controls{sizeof(controls), ICC_LISTVIEW_CLASSES | ICC_STANDARD_CLASSES};
    InitCommonControlsEx(&controls);

    g_registry_path = registry_path();
    g_status_directory = workdir_root() / L"status";
    std::filesystem::create_directories(g_status_directory);
    std::string error;
    std::filesystem::path load_path = g_registry_path;
    if (!std::filesystem::exists(load_path) && std::filesystem::exists(example_registry_path())) {
        load_path = example_registry_path();
    }
    if (std::filesystem::exists(load_path)) {
        if (!g_registry.load(load_path, error)) {
            MessageBoxW(nullptr, widen(error).c_str(), L"配置加载失败", MB_ICONERROR | MB_OK);
        }
    }

    WNDCLASSEXW window_class{};
    window_class.cbSize = sizeof(window_class);
    window_class.style = CS_HREDRAW | CS_VREDRAW;
    window_class.lpfnWndProc = window_proc;
    window_class.hInstance = instance;
    window_class.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    window_class.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    window_class.lpszClassName = L"JakaRobotManagerWindow";
    if (!RegisterClassExW(&window_class)) return 1;

    HWND window = CreateWindowExW(0, window_class.lpszClassName, L"JAKA 多机器人控制台",
                                  WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                                  CW_USEDEFAULT, CW_USEDEFAULT, 1320, 1120,
                                  nullptr, nullptr, instance, nullptr);
    if (!window) return 2;
    ShowWindow(window, show_command);
    UpdateWindow(window);

    MSG message{};
    while (GetMessageW(&message, nullptr, 0, 0) > 0) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    return static_cast<int>(message.wParam);
}
