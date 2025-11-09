#pragma once

#include <string>
#include <memory>

// Forward declarations
namespace bt {
class Action;
}

class IMavrosAdapter;

/**
 * @brief 动作处理器接口：定义所有动作处理器的通用协议
 * 
 * 此接口将具体的动作实现逻辑从 BehaviorExecutive 中分离出来，
 * 使得每个动作可以独立开发、测试和维护。
 */
class IActionHandler {
public:
    virtual ~IActionHandler() = default;

    /**
     * @brief 动作上下文：包含执行动作所需的环境信息
     */
    struct Context {
        double current_latitude;     // 当前无人机纬度（度）
        double current_longitude;    // 当前无人机经度（度）
        double target_altitude;      // 目标飞行高度（米）
        std::string current_mode;    // 当前飞行模式
        
        // 可以根据需要添加更多上下文信息
        // double current_relative_altitude;
        // bool is_armed;
    };

    /**
     * @brief 当动作被激活时调用
     * 
     * 此方法在动作状态变为 active 并且 active_has_changed() 返回 true 时调用。
     * 处理器应该在此方法中执行动作的主要逻辑。
     * 
     * @param action 被激活的动作指针
     * @param ctx 当前环境上下文
     */
    virtual void on_activated(bt::Action* action, const Context& ctx) = 0;

    /**
     * @brief 检查执行动作前的安全条件
     * 
     * 此方法在动作激活前调用，用于验证是否满足执行条件。
     * 例如：GPS 有效性、电池电量、高度限制等。
     * 
     * @param ctx 当前环境上下文
     * @return true 如果所有安全检查通过
     */
    virtual bool check_safety(const Context& ctx) = 0;
};




