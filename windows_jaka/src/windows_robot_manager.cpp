#define UNICODE
#define _UNICODE
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commctrl.h>

#include "control_pipe.hpp"
#include "robot_registry.hpp"
#include "runtime_plan.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#pragma comment(lib, "comctl32.lib")

namespace {

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
constexpr UINT_PTR ID_SESSION_TIMER = 1;

HWND g_window{};
HWND g_status{};
HWND g_robot_list{};
HWND g_group_list{};
HWND g_session_status{};
HWND g_status_list{};
HANDLE g_session_process{};
HANDLE g_session_job{};
std::filesystem::path g_status_directory;
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

void add_control(HWND parent, const wchar_t* klass, const wchar_t* text, DWORD style,
                 int x, int y, int width, int height, int id,
                 std::vector<HWND>* group = nullptr) {
    HWND control = CreateWindowExW(0, klass, text, style | WS_CHILD | WS_VISIBLE,
                                   x, y, width, height, parent,
                                   reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
                                   GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT,
                 reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), TRUE);
    if (group) group->push_back(control);
}

void add_label(HWND parent, const wchar_t* text, int x, int y, int width,
               std::vector<HWND>* group = nullptr) {
    add_control(parent, L"STATIC", text, SS_LEFT, x, y, width, 22, 0, group);
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
void update_session_ui();
void refresh_status_list();
std::filesystem::path workdir_root();

void add_edit(HWND parent, int id, int x, int y, int width,
              std::vector<HWND>* group = nullptr) {
    add_control(parent, L"EDIT", L"", WS_BORDER | ES_AUTOHSCROLL,
                x, y, width, 28, id, group);
}

void build_ui(HWND window) {
    add_control(window, L"STATIC", L"JAKA 多机器人管理", SS_LEFT,
                20, 10, 500, 32, 0);
    g_status = CreateWindowExW(0, L"STATIC", L"", WS_CHILD | WS_VISIBLE | SS_LEFT,
                               550, 10, 620, 28, window,
                               reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_STATUS)),
                               GetModuleHandleW(nullptr), nullptr);
    SendMessageW(g_status, WM_SETFONT, reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), TRUE);

    add_label(window, L"机器人列表", 20, 48, 300);
    g_robot_list = CreateWindowExW(WS_EX_CLIENTEDGE, WC_LISTVIEWW, L"",
                                   WS_CHILD | WS_VISIBLE | LVS_REPORT | LVS_SINGLESEL,
                                   20, 74, 340, 420, window,
                                   reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_ROBOT_LIST)),
                                   GetModuleHandleW(nullptr), nullptr);
    ListView_SetExtendedListViewStyle(g_robot_list, LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);
    add_column(g_robot_list, 0, 95, L"ID");
    add_column(g_robot_list, 1, 120, L"名称");
    add_column(g_robot_list, 2, 110, L"IP");

    add_label(window, L"机器人参数", 390, 48, 300, &g_robot_controls);
    add_label(window, L"ID", 390, 78, 100, &g_robot_controls);
    add_edit(window, ID_ROBOT_ID, 490, 74, 240, &g_robot_controls);
    add_label(window, L"名称", 750, 78, 100, &g_robot_controls);
    add_edit(window, ID_ROBOT_NAME, 850, 74, 300, &g_robot_controls);
    add_label(window, L"型号", 390, 118, 100, &g_robot_controls);
    add_edit(window, ID_ROBOT_MODEL, 490, 114, 240, &g_robot_controls);
    add_label(window, L"IP", 750, 118, 100, &g_robot_controls);
    add_edit(window, ID_ROBOT_IP, 850, 114, 300, &g_robot_controls);
    add_control(window, L"BUTTON", L"启用", BS_AUTOCHECKBOX,
                390, 154, 120, 28, ID_ROBOT_ENABLED, &g_robot_controls);
    add_label(window, L"滤波器", 520, 158, 90, &g_robot_controls);
    HWND filter = CreateWindowExW(0, L"COMBOBOX", L"",
                                  WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST,
                                  610, 154, 140, 160, window,
                                  reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_ROBOT_FILTER)),
                                  GetModuleHandleW(nullptr), nullptr);
    SendMessageW(filter, WM_SETFONT, reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), TRUE);
    SendMessageW(filter, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"none"));
    SendMessageW(filter, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"lpf"));
    SendMessageW(filter, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"nlf"));
    g_robot_controls.push_back(filter);

    add_label(window, L"LPF 截止", 760, 158, 90, &g_robot_controls);
    add_edit(window, ID_ROBOT_LPF, 850, 154, 120, &g_robot_controls);
    add_label(window, L"最大速度 rad/s", 390, 198, 150, &g_robot_controls);
    add_edit(window, ID_ROBOT_VEL, 540, 194, 140, &g_robot_controls);
    add_label(window, L"最大加速度", 700, 198, 130, &g_robot_controls);
    add_edit(window, ID_ROBOT_ACC, 830, 194, 140, &g_robot_controls);
    add_label(window, L"关节方向", 390, 238, 120, &g_robot_controls);
    add_edit(window, ID_ROBOT_DIRECTION, 500, 234, 650, &g_robot_controls);
    add_label(window, L"下限角度", 390, 278, 120, &g_robot_controls);
    add_edit(window, ID_ROBOT_LOWER, 500, 274, 650, &g_robot_controls);
    add_label(window, L"上限角度", 390, 318, 120, &g_robot_controls);
    add_edit(window, ID_ROBOT_UPPER, 500, 314, 650, &g_robot_controls);
    add_label(window, L"安全姿态角度", 390, 358, 140, &g_robot_controls);
    add_edit(window, ID_ROBOT_SAFE, 530, 354, 620, &g_robot_controls);
    add_control(window, L"BUTTON", L"新增机器人", WS_TABSTOP | BS_PUSHBUTTON,
                390, 400, 120, 34, ID_ROBOT_ADD, &g_robot_controls);
    add_control(window, L"BUTTON", L"应用修改", WS_TABSTOP | BS_PUSHBUTTON,
                520, 400, 120, 34, ID_ROBOT_APPLY, &g_robot_controls);
    add_control(window, L"BUTTON", L"删除选中", WS_TABSTOP | BS_PUSHBUTTON,
                650, 400, 120, 34, ID_ROBOT_DELETE, &g_robot_controls);
    add_control(window, L"BUTTON", L"保存配置", WS_TABSTOP | BS_PUSHBUTTON,
                800, 400, 120, 34, ID_SAVE, &g_robot_controls);
    add_control(window, L"BUTTON", L"重新加载", WS_TABSTOP | BS_PUSHBUTTON,
                930, 400, 120, 34, ID_RELOAD, &g_robot_controls);

    add_label(window, L"遥操作组", 20, 520, 300);
    g_group_list = CreateWindowExW(WS_EX_CLIENTEDGE, WC_LISTVIEWW, L"",
                                   WS_CHILD | WS_VISIBLE | LVS_REPORT | LVS_SINGLESEL,
                                   20, 546, 340, 260, window,
                                   reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_GROUP_LIST)),
                                   GetModuleHandleW(nullptr), nullptr);
    ListView_SetExtendedListViewStyle(g_group_list, LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);
    add_column(g_group_list, 0, 100, L"ID");
    add_column(g_group_list, 1, 220, L"名称");

    add_label(window, L"组参数", 390, 520, 300, &g_group_controls);
    add_label(window, L"组 ID", 390, 552, 100, &g_group_controls);
    add_edit(window, ID_GROUP_ID, 490, 548, 240, &g_group_controls);
    add_label(window, L"组名称", 750, 552, 100, &g_group_controls);
    add_edit(window, ID_GROUP_NAME, 850, 548, 300, &g_group_controls);
    add_label(window, L"操作臂 ID", 390, 592, 120, &g_group_controls);
    add_edit(window, ID_GROUP_OPERATOR, 510, 588, 220, &g_group_controls);
    add_label(window, L"跟随臂 ID，逗号分隔", 390, 632, 220, &g_group_controls);
    add_edit(window, ID_GROUP_FOLLOWERS, 610, 628, 540, &g_group_controls);
    add_control(window, L"BUTTON", L"新增组", WS_TABSTOP | BS_PUSHBUTTON,
                390, 680, 120, 34, ID_GROUP_ADD, &g_group_controls);
    add_control(window, L"BUTTON", L"应用组修改", WS_TABSTOP | BS_PUSHBUTTON,
                520, 680, 120, 34, ID_GROUP_APPLY, &g_group_controls);
    add_control(window, L"BUTTON", L"删除组", WS_TABSTOP | BS_PUSHBUTTON,
                650, 680, 120, 34, ID_GROUP_DELETE, &g_group_controls);
    add_control(window, L"BUTTON", L"真实运动授权", BS_AUTOCHECKBOX,
                390, 730, 150, 28, ID_GROUP_REAL_MOTION, &g_group_controls);
    add_control(window, L"BUTTON", L"启动选中组", WS_TABSTOP | BS_PUSHBUTTON,
                550, 726, 120, 34, ID_GROUP_START, &g_group_controls);
    add_control(window, L"BUTTON", L"停止当前组", WS_TABSTOP | BS_PUSHBUTTON,
                680, 726, 120, 34, ID_GROUP_STOP, &g_group_controls);
    g_session_status = CreateWindowExW(0, L"STATIC", L"会话未启动",
                                       WS_CHILD | WS_VISIBLE | SS_LEFT,
                                       820, 734, 330, 26, window,
                                       reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_SESSION_STATUS)),
                                       GetModuleHandleW(nullptr), nullptr);
    SendMessageW(g_session_status, WM_SETFONT,
                 reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), TRUE);

    add_label(window, L"机器人实时状态", 20, 820, 300);
    g_status_list = CreateWindowExW(WS_EX_CLIENTEDGE, WC_LISTVIEWW, L"",
                                    WS_CHILD | WS_VISIBLE | LVS_REPORT | LVS_SINGLESEL,
                                    20, 846, 1140, 180, window,
                                    reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_STATUS_LIST)),
                                    GetModuleHandleW(nullptr), nullptr);
    ListView_SetExtendedListViewStyle(g_status_list, LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);
    add_column(g_status_list, 0, 110, L"机器人");
    add_column(g_status_list, 1, 110, L"模式");
    add_column(g_status_list, 2, 70, L"连接");
    add_column(g_status_list, 3, 70, L"上电");
    add_column(g_status_list, 4, 70, L"使能");
    add_column(g_status_list, 5, 70, L"拖动");
    add_column(g_status_list, 6, 70, L"伺服");
    add_column(g_status_list, 7, 170, L"故障");
    add_column(g_status_list, 8, 90, L"数据年龄ms");
    add_column(g_status_list, 9, 90, L"序列");

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
    const auto root = workdir_root();
    const auto candidate = root / L"windows_jaka" / L"run_group_from_registry.ps1";
    if (std::filesystem::exists(candidate)) return candidate;
    wchar_t module[MAX_PATH]{};
    GetModuleFileNameW(nullptr, module, MAX_PATH);
    return std::filesystem::path(module).parent_path() / L"run_group_from_registry.ps1";
}

bool session_running() {
    if (!g_session_process) return false;
    return WaitForSingleObject(g_session_process, 0) == WAIT_TIMEOUT;
}

void update_session_ui() {
    const bool running = session_running();
    EnableWindow(GetDlgItem(g_window, ID_GROUP_START), running ? FALSE : TRUE);
    EnableWindow(GetDlgItem(g_window, ID_GROUP_STOP), running ? TRUE : FALSE);
    EnableWindow(GetDlgItem(g_window, ID_GROUP_REAL_MOTION), running ? FALSE : TRUE);
    if (g_session_status && !running) {
        SetWindowTextW(g_session_status, L"会话未启动");
    }
}

void start_selected_group() {
    if (session_running()) {
        set_status(L"当前已有组会话在运行");
        return;
    }
    const int index = selected_index(g_group_list);
    if (index < 0 || index >= static_cast<int>(g_registry.groups.size())) {
        set_status(L"请先选择一个遥操作组");
        return;
    }
    if (!save_registry()) return;

    windows_jaka::TeleopRuntimePlan plan;
    std::string error;
    const std::string group_id = g_registry.groups[static_cast<std::size_t>(index)].id;
    if (!windows_jaka::build_teleop_runtime_plan(g_registry, group_id, 30101, plan, error)) {
        set_status(L"运行计划无效：" + widen(error));
        return;
    }

    const auto launcher = group_launcher_path();
    if (!std::filesystem::exists(launcher)) {
        set_status(L"找不到组启动脚本：" + launcher.wstring());
        return;
    }
    const bool real_motion =
        SendMessageW(GetDlgItem(g_window, ID_GROUP_REAL_MOTION), BM_GETCHECK, 0, 0) == BST_CHECKED;
    std::wstring command = L"powershell.exe -NoProfile -ExecutionPolicy Bypass -File " +
        quote_w(launcher.wstring()) +
        L" -RegistryPath " + quote_w(g_registry_path.wstring()) +
        L" -GroupId " + quote_w(widen(group_id)) +
        L" -BasePort 30101 -StatusDirectory " + quote_w(g_status_directory.wstring());
    command += real_motion ? L" -ArmMotion" : L" -DryRun";

    std::error_code ignored;
    std::filesystem::remove(g_status_directory / (widen(plan.operator_robot.id) + L".status"), ignored);
    for (const auto& follower : plan.follower_robots) {
        std::filesystem::remove(g_status_directory / (widen(follower.id) + L".status"), ignored);
    }

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

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION process{};
    const BOOL created = CreateProcessW(nullptr, command_line.data(), nullptr, nullptr, FALSE,
                                        CREATE_NO_WINDOW | CREATE_NEW_PROCESS_GROUP,
                                        nullptr, workdir_root().c_str(), &startup, &process);
    const DWORD create_error = created ? ERROR_SUCCESS : GetLastError();
    SetEnvironmentVariableW(L"JAKA_ENABLE_MOTION", old_motion.empty() ? nullptr : old_motion.c_str());

    if (!created) {
        set_status(L"启动组失败，Windows 错误码=" + std::to_wstring(create_error));
        return;
    }

    CloseHandle(process.hThread);
    g_session_process = process.hProcess;
    g_session_job = CreateJobObjectW(nullptr, nullptr);
    if (g_session_job) {
        JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
        limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
        if (!SetInformationJobObject(g_session_job, JobObjectExtendedLimitInformation,
                                     &limits, sizeof(limits)) ||
            !AssignProcessToJobObject(g_session_job, g_session_process)) {
            CloseHandle(g_session_job);
            g_session_job = nullptr;
        }
    }
    if (g_session_status) {
        SetWindowTextW(g_session_status, L"组会话运行中");
    }
    set_status((real_motion ? L"真实运动组已启动：" : L"Dry-run 组已启动：") +
               widen(group_id) + L"，跟随臂=" + std::to_wstring(plan.follower_endpoints.size()));
    update_session_ui();
}

void stop_selected_group() {
    if (!g_session_process) return;
    windows_jaka::TeleopRuntimePlan plan;
    std::string error;
    const int index = selected_index(g_group_list);
    if (index >= 0 && index < static_cast<int>(g_registry.groups.size())) {
        const std::string group_id = g_registry.groups[static_cast<std::size_t>(index)].id;
        if (windows_jaka::build_teleop_runtime_plan(g_registry, group_id, 30101, plan, error)) {
            windows_jaka::send_control_command(widen(plan.operator_control_pipe), "STOP");
            for (const auto& endpoint : plan.follower_endpoints) {
                windows_jaka::send_control_command(widen(endpoint.control_pipe), "STOP");
            }
        }
    }
    if (WaitForSingleObject(g_session_process, 3000) == WAIT_TIMEOUT) {
        if (g_session_job) TerminateJobObject(g_session_job, 1);
        else TerminateProcess(g_session_process, 1);
        WaitForSingleObject(g_session_process, 1000);
    }
    if (g_session_job) CloseHandle(g_session_job);
    if (g_session_process) CloseHandle(g_session_process);
    g_session_job = nullptr;
    g_session_process = nullptr;
    if (g_session_status) SetWindowTextW(g_session_status, L"会话已停止");
    set_status(L"组会话已停止");
    update_session_ui();
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
    case ID_ROBOT_APPLY: {
        const int index = selected_index(g_robot_list);
        windows_jaka::RobotProfile robot;
        std::string error;
        if (index < 0 || !read_robot_form(robot, error)) {
            set_status(L"机器人参数无效：" + widen(error));
            break;
        }
        g_registry.robots[static_cast<std::size_t>(index)] = robot;
        fill_robot_list();
        select_list_item(g_robot_list, index);
        break;
    }
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
    case ID_GROUP_APPLY: {
        const int index = selected_index(g_group_list);
        windows_jaka::TeleopGroup group;
        std::string error;
        if (index < 0 || !read_group_form(group, error)) {
            set_status(L"组参数无效：" + widen(error));
            break;
        }
        g_registry.groups[static_cast<std::size_t>(index)] = group;
        fill_group_list();
        select_list_item(g_group_list, index);
        break;
    }
    case ID_GROUP_DELETE: {
        const int index = selected_index(g_group_list);
        if (index < 0) break;
        g_registry.groups.erase(g_registry.groups.begin() + index);
        fill_group_list();
        break;
    }
    case ID_SAVE:
        save_registry();
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
    case WM_TIMER:
        if (wparam == ID_SESSION_TIMER) {
            refresh_status_list();
            if (g_session_process && !session_running()) {
                if (g_session_job) CloseHandle(g_session_job);
                if (g_session_process) CloseHandle(g_session_process);
                g_session_job = nullptr;
                g_session_process = nullptr;
                if (g_session_status) SetWindowTextW(g_session_status, L"会话进程已退出");
                update_session_ui();
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
        stop_selected_group();
        KillTimer(window, ID_SESSION_TIMER);
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

    HWND window = CreateWindowExW(0, window_class.lpszClassName, L"JAKA 多机器人管理",
                                  WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                                  CW_USEDEFAULT, CW_USEDEFAULT, 1220, 1100,
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
