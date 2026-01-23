#include "BTComm.h"
#include "Uart_Config.h"
#include "Motor_ctr.h"
#include "M3PWM.h"
#include "PID_Controller.h"
#include "BlackPoint_Finder.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*=== 接收缓冲区和状态 ===*/
static char g_cmd_buffer[BT_CMD_BUFFER_SIZE];
static uint8_t g_cmd_index = 0;

/*=== 当前运动状态 ===*/
static Motion_Mode_t g_motion_mode = MOTION_IDLE;
static uint8_t g_motion_speed = 0;        // 当前运动速度 0-100
static uint8_t g_base_speed = 30;         // 基础速度（默认30%）
static uint8_t g_vacuum_enabled = 0;      // 吸盘是否启用
static uint8_t g_vacuum_speed = 20;       // 吸盘转速 0-100
static uint8_t g_key_control_mode = 0;    // 键控模式标志 0=OFF, 1=ON
static uint8_t g_emergency_stop = 0;      // 急停标志 0=正常, 1=急停

/*=== 吸盘默认转速配置 ===*/
#define VACUUM_DEFAULT_SPEED  20          // 吸盘默认转速百分比

/**
 * @brief 初始化蓝牙通信模块
 */
void BT_Init(void)
{
    g_cmd_index = 0;
    memset(g_cmd_buffer, 0, sizeof(g_cmd_buffer));
    g_motion_mode = MOTION_IDLE;
    g_motion_speed = 0;
    g_base_speed = 30;
    g_vacuum_enabled = 0;
    g_vacuum_speed = VACUUM_DEFAULT_SPEED;
    g_key_control_mode = 0;
    g_emergency_stop = 0;
}

/**
 * @brief 发送响应字符串
 */
static void BT_SendResponse(const char *str)
{
    Uart2_SendBuf((uint8_t*)str, strlen(str));
}

/**
 * @brief 处理急停命令（全局生效）
 */
void BT_EmergencyStop(void)
{
    g_emergency_stop = 1;
    g_motion_mode = MOTION_IDLE;
    g_motion_speed = 0;
    
    // 立即停止所有电机
    Motor_StopAll();
    Motor_Disable();
    
    // 停止负压电机
    M3PWM_SetDutyCycle(0);
    M3PWM_Stop();
    g_vacuum_enabled = 0;
    
    BT_SendResponse("OK:ESTOP\r\n");
}

/**
 * @brief 清除急停状态
 */
void BT_ClearEmergencyStop(void)
{
    g_emergency_stop = 0;
    BT_SendResponse("OK:ESTOP_CLR\r\n");
}

/**
 * @brief 检查是否处于急停状态
 */
uint8_t BT_IsEmergencyStopped(void)
{
    return g_emergency_stop;
}

/**
 * @brief 负压电机控制（全局生效）
 */
void BT_VacuumControl(uint8_t enable)
{
    g_vacuum_enabled = (enable > 0) ? 1 : 0;
    
    if(g_vacuum_enabled)
    {
        // 确保有转速设置
        if(g_vacuum_speed == 0)
        {
            g_vacuum_speed = VACUUM_DEFAULT_SPEED;
        }
        uint16_t duty = (1000 * g_vacuum_speed) / 100;
        M3PWM_SetDutyCycle(duty);
        M3PWM_Start();
        BT_SendResponse("OK:VON\r\n");
    }
    else
    {
        M3PWM_SetDutyCycle(0);
        M3PWM_Stop();
        BT_SendResponse("OK:VOFF\r\n");
    }
}

/**
 * @brief 设置负压电机转速（全局生效）
 */
void BT_VacuumSetSpeed(uint8_t speed)
{
    if(speed > 100) speed = 100;
    g_vacuum_speed = speed;
    
    if(g_vacuum_enabled)
    {
        uint16_t duty = (1000 * speed) / 100;
        M3PWM_SetDutyCycle(duty);
    }
    
    char buf[32];
    sprintf(buf, "OK:VS=%d\r\n", speed);
    BT_SendResponse(buf);
}

/**
 * @brief 处理停止移动命令
 */
static void BT_HandleStop(void)
{
    g_motion_mode = MOTION_IDLE;
    g_motion_speed = 0;
    BT_SendResponse("OK:STOP\r\n");
}

/**
 * @brief 处理前进命令
 */
static void BT_HandleMoveForward(uint8_t speed)
{
    // 检查急停状态
    if(g_emergency_stop)
    {
        BT_SendResponse("ERR:ESTOP_ACTIVE\r\n");
        return;
    }
    
    // 检查键控模式（方向控制需要在键控模式下）
    if(!g_key_control_mode)
    {
        BT_SendResponse("ERR:NOT_KEY_MODE\r\n");
        return;
    }
    
    g_motion_mode = MOTION_FORWARD;
    g_motion_speed = (speed > 0) ? speed : g_base_speed;
    
    char buf[32];
    sprintf(buf, "OK:FWD=%d\r\n", g_motion_speed);
    BT_SendResponse(buf);
}

/**
 * @brief 处理后退命令
 */
static void BT_HandleMoveBackward(uint8_t speed)
{
    if(g_emergency_stop)
    {
        BT_SendResponse("ERR:ESTOP_ACTIVE\r\n");
        return;
    }
    
    if(!g_key_control_mode)
    {
        BT_SendResponse("ERR:NOT_KEY_MODE\r\n");
        return;
    }
    
    g_motion_mode = MOTION_BACKWARD;
    g_motion_speed = (speed > 0) ? speed : g_base_speed;
    
    char buf[32];
    sprintf(buf, "OK:BWD=%d\r\n", g_motion_speed);
    BT_SendResponse(buf);
}

/**
 * @brief 处理左转命令
 */
static void BT_HandleTurnLeft(uint8_t speed)
{
    if(g_emergency_stop)
    {
        BT_SendResponse("ERR:ESTOP_ACTIVE\r\n");
        return;
    }
    
    if(!g_key_control_mode)
    {
        BT_SendResponse("ERR:NOT_KEY_MODE\r\n");
        return;
    }
    
    g_motion_mode = MOTION_TURN_LEFT;
    g_motion_speed = (speed > 0) ? speed : g_base_speed;
    
    char buf[32];
    sprintf(buf, "OK:TL=%d\r\n", g_motion_speed);
    BT_SendResponse(buf);
}

/**
 * @brief 处理右转命令
 */
static void BT_HandleTurnRight(uint8_t speed)
{
    if(g_emergency_stop)
    {
        BT_SendResponse("ERR:ESTOP_ACTIVE\r\n");
        return;
    }
    
    if(!g_key_control_mode)
    {
        BT_SendResponse("ERR:NOT_KEY_MODE\r\n");
        return;
    }
    
    g_motion_mode = MOTION_TURN_RIGHT;
    g_motion_speed = (speed > 0) ? speed : g_base_speed;
    
    char buf[32];
    sprintf(buf, "OK:TR=%d\r\n", g_motion_speed);
    BT_SendResponse(buf);
}

/**
 * @brief 设置基础速度
 */
static void BT_HandleSetBaseSpeed(uint8_t speed)
{
    if(speed > 250) speed = 250;
    g_base_speed = speed;
    
    char buf[32];
    sprintf(buf, "OK:SPD=%d\r\n", speed);
    BT_SendResponse(buf);
}

/**
 * @brief 查询当前状态
 */
static void BT_HandleQueryStatus(void)
{
    char buffer[128];
    
    sprintf(buffer, "MODE:%s,SPD:%d,BASE:%d,VAC:%s,VS:%d,KEY:%s,ESTOP:%s\r\n",
            (g_motion_mode == MOTION_IDLE) ? "IDLE" :
            (g_motion_mode == MOTION_FORWARD) ? "FWD" :
            (g_motion_mode == MOTION_BACKWARD) ? "BWD" :
            (g_motion_mode == MOTION_TURN_LEFT) ? "TL" : "TR",
            g_motion_speed,
            g_base_speed,
            g_vacuum_enabled ? "ON" : "OFF",
            g_vacuum_speed,
            g_key_control_mode ? "ON" : "OFF",
            g_emergency_stop ? "YES" : "NO");
    
    BT_SendResponse(buffer);
}

/**
 * @brief 处理键控模式开关
 */
static void BT_HandleKeyControlMode(uint8_t enabled)
{
    g_key_control_mode = (enabled > 0) ? 1 : 0;
    
    if(g_key_control_mode)
    {
        // 进入键控模式，清除急停，停止当前运动
        g_emergency_stop = 0;
        g_motion_mode = MOTION_IDLE;
        g_motion_speed = 0;
        BT_SendResponse("OK:KEY=ON\r\n");
    }
    else
    {
        // 退出键控模式，停止运动
        g_motion_mode = MOTION_IDLE;
        g_motion_speed = 0;
        BT_SendResponse("OK:KEY=OFF\r\n");
    }
}

/**
 * @brief 处理键控模式切换（硬件K2直接调用）
 */
void BT_HandleKeyControlModeChange(uint8_t enabled)
{
    BT_HandleKeyControlMode(enabled);
}

/**
 * @brief 解析并执行字符串命令
 * @param cmd: 命令字符串（不含换行符）
 */
static void BT_ParseCommand(const char *cmd)
{
    // 去掉可能的空格
    while(*cmd == ' ') cmd++;
    
    // 检查空命令
    if(strlen(cmd) == 0) return;
    
    // 解析参数（如果有的话）
    int param = 0;
    char cmd_part[16] = {0};
    const char *colon = strchr(cmd, ':');
    
    if(colon != NULL)
    {
        // 有参数的命令
        int cmd_len = colon - cmd;
        if(cmd_len > 15) cmd_len = 15;
        strncpy(cmd_part, cmd, cmd_len);
        cmd_part[cmd_len] = 0;
        param = atoi(colon + 1);
    }
    else
    {
        // 无参数的命令
        strncpy(cmd_part, cmd, 15);
    }
    
    // 转换为大写方便比较
    for(int i = 0; cmd_part[i]; i++)
    {
        if(cmd_part[i] >= 'a' && cmd_part[i] <= 'z')
        {
            cmd_part[i] -= 32;
        }
    }
    
    // ========== 全局命令（任何时候都生效） ==========
    
    // 急停命令 - 最高优先级
    if(strcmp(cmd_part, "ESTOP") == 0 || strcmp(cmd_part, "E") == 0)
    {
        BT_EmergencyStop();
        return;
    }
    
    // 清除急停
    if(strcmp(cmd_part, "CLR") == 0 || strcmp(cmd_part, "CLEAR") == 0)
    {
        BT_ClearEmergencyStop();
        return;
    }
    
    // 负压开
    if(strcmp(cmd_part, "VON") == 0)
    {
        BT_VacuumControl(1);
        return;
    }
    
    // 负压关
    if(strcmp(cmd_part, "VOFF") == 0)
    {
        BT_VacuumControl(0);
        return;
    }
    
    // 负压转速
    if(strcmp(cmd_part, "VS") == 0)
    {
        BT_VacuumSetSpeed((uint8_t)param);
        return;
    }
    
    // 基础速度设置
    if(strcmp(cmd_part, "SPD") == 0)
    {
        BT_HandleSetBaseSpeed((uint8_t)param);
        return;
    }

    // PID参数设置：PID:kp,ki,kd[,kf]
    if(strcmp(cmd_part, "PID") == 0)
    {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, kf = 0.0f;
        int args = 0;
        if(colon)
        {
             args = sscanf(colon + 1, "%f,%f,%f,%f", &kp, &ki, &kd, &kf);
        }

        if(args == 3)
        {
            SpeedPID_SetParamBoth(kp, ki, kd);
            SpeedPID_ResetState();
            char buf[48];
            sprintf(buf, "OK:PID=%.2f,%.2f,%.2f\r\n", kp, ki, kd);
            BT_SendResponse(buf);
        }
        else if(args == 4)
        {
            SpeedPID_SetParamBoth(kp, ki, kd);
            SpeedPID_SetFeedForwardBoth(kf);
            SpeedPID_ResetState();
            char buf[64];
            sprintf(buf, "OK:PID=%.2f,%.2f,%.2f,KF=%.2f\r\n", kp, ki, kd, kf);
            BT_SendResponse(buf);
        }
        else
        {
            BT_SendResponse("ERR:PID_FMT\r\n");
        }
        return;
    }
    
    // 位置环PPID参数设置：PPID:kp,ki,kd,gyro_kd
    if(strcmp(cmd_part, "PPID") == 0)
    {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, gyro_kd = 0.0f;
        int args = 0;
        if(colon)
        {
             args = sscanf(colon + 1, "%f,%f,%f,%f", &kp, &ki, &kd, &gyro_kd);
        }

        if(args == 4)
        {
            PositionPID_SetGlobalParams(kp, ki, kd, gyro_kd);
            char buf[64];
            sprintf(buf, "OK:PPID=%.1f,%.1f,%.1f,%.1f\r\n", kp, ki, kd, gyro_kd);
            BT_SendResponse(buf);
        }
        else
        {
            BT_SendResponse("ERR:PPID_FMT\r\n");
        }
        return;
    }
    
    // 单独设置 KF
    if(strcmp(cmd_part, "KF") == 0)
    {
        float kf = 0.0f;
        if(colon && (sscanf(colon + 1, "%f", &kf) == 1))
        {
            SpeedPID_SetFeedForwardBoth(kf);
            char buf[32];
            sprintf(buf, "OK:KF=%.2f\r\n", kf);
            BT_SendResponse(buf);
        }
        else
        {
             BT_SendResponse("ERR:KF_FMT\r\n");
        }
        return;
    }
    
    // 陀螺仪阻尼开关: GYRO:1=开启, GYRO:0=关闭（架空测试时禁用）
    if(strcmp(cmd_part, "GYRO") == 0)
    {
        PID_GyroDamping_Enable((uint8_t)param);
        char buf[32];
        sprintf(buf, "OK:GYRO=%s\r\n", PID_GyroDamping_IsEnabled() ? "ON" : "OFF");
        BT_SendResponse(buf);
        return;
    }
    
    // 模拟黑点测试命令: BLK:1,2,3 表示1,2,3路检测到黑线
    // BLK:OFF 关闭模拟模式
    if(strcmp(cmd_part, "BLK") == 0)
    {
        if(colon)
        {
            const char *params = colon + 1;
            
            // 检查是否关闭模拟模式
            if(strcmp(params, "OFF") == 0 || strcmp(params, "off") == 0 || strcmp(params, "0") == 0)
            {
                BlackPoint_Finder_SetSimulateMode(0);
                BT_SendResponse("OK:BLK=OFF\r\n");
            }
            else
            {
                // 解析通道号列表: 1,2,3 或 9,10
                uint8_t channels[16];
                uint8_t count = 0;
                char buf_parse[64];
                char *token;
                
                // 复制字符串以避免修改原始数据
                strncpy(buf_parse, params, sizeof(buf_parse) - 1);
                buf_parse[sizeof(buf_parse) - 1] = '\0';
                
                // 使用逗号分割
                token = strtok(buf_parse, ",");
                while(token != NULL && count < 16)
                {
                    int ch = atoi(token);
                    if(ch >= 0 && ch < 16)
                    {
                        channels[count++] = (uint8_t)ch;
                    }
                    token = strtok(NULL, ",");
                }
                
                if(count > 0)
                {
                    BlackPoint_Finder_SetSimulateChannels(channels, count);
                    BlackPoint_Finder_SetSimulateMode(1);
                    
                    char resp[64];
                    sprintf(resp, "OK:BLK=%d ch\r\n", count);
                    BT_SendResponse(resp);
                }
                else
                {
                    BT_SendResponse("ERR:BLK_FMT\r\n");
                }
            }
        }
        else
        {
            // 无参数，启用默认模拟模式（中间两路7,8）
            BlackPoint_Finder_SetSimulateMode(1);
            BT_SendResponse("OK:BLK=7,8(default)\r\n");
        }
        return;
    }
    
    // 状态查询
    if(strcmp(cmd_part, "?STATUS") == 0 || strcmp(cmd_part, "STATUS") == 0 || strcmp(cmd_part, "?") == 0)
    {
        BT_HandleQueryStatus();
        return;
    }
    
    // 键控模式开关
    if(strcmp(cmd_part, "KEY") == 0)
    {
        BT_HandleKeyControlMode((uint8_t)param);
        return;
    }
    
    // ========== 键控模式命令（需要在键控模式下） ==========
    
    // 前进
    if(strcmp(cmd_part, "FWD") == 0 || strcmp(cmd_part, "W") == 0)
    {
        BT_HandleMoveForward((uint8_t)param);
        return;
    }
    
    // 后退
    if(strcmp(cmd_part, "BWD") == 0 || strcmp(cmd_part, "S") == 0)
    {
        BT_HandleMoveBackward((uint8_t)param);
        return;
    }
    
    // 左转
    if(strcmp(cmd_part, "TL") == 0 || strcmp(cmd_part, "A") == 0)
    {
        BT_HandleTurnLeft((uint8_t)param);
        return;
    }
    
    // 右转
    if(strcmp(cmd_part, "TR") == 0 || strcmp(cmd_part, "D") == 0)
    {
        BT_HandleTurnRight((uint8_t)param);
        return;
    }
    
    // 停止移动
    if(strcmp(cmd_part, "STOP") == 0 || strcmp(cmd_part, "X") == 0)
    {
        BT_HandleStop();
        return;
    }
    
    // 帮助命令
    if(strcmp(cmd_part, "HELP") == 0 || strcmp(cmd_part, "H") == 0)
    {
        BT_SendResponse("=== BT Commands ===\r\n");
        BT_SendResponse("Global: ESTOP,CLR,VON,VOFF,VS:xx,SPD:xx,?\r\n");
        BT_SendResponse("KeyCtrl: FWD,BWD,TL,TR,STOP (or W,S,A,D,X)\r\n");
        BT_SendResponse("KEY:1=Enter KeyMode, KEY:0=Exit\r\n");
        return;
    }
    
    // 未知命令
    BT_SendResponse("ERR:UNKNOWN_CMD\r\n");
}

/**
 * @brief 处理接收的数据（字符串解析）
 *        改为非阻塞方式，避免死循环
 */
void BT_Process(void)
{
    // 限制单次处理的最大字节数，避免长时间阻塞主循环
    uint8_t max_bytes = 32;
    uint8_t valid = 0;
    
    while(max_bytes > 0)
    {
        uint8_t byte = Uart2_ReadByte(&valid);
        if(!valid)
        {
            break; // 没有数据了，退出
        }
        max_bytes--;
        
        // 检测换行符，表示命令结束
        if(byte == '\n' || byte == '\r')
        {
            if(g_cmd_index > 0)
            {
                g_cmd_buffer[g_cmd_index] = '\0';
                BT_ParseCommand(g_cmd_buffer);
                g_cmd_index = 0;
            }
        }
        else if(byte >= 32 && byte < 127)  // 可打印字符
        {
            if(g_cmd_index < BT_CMD_BUFFER_SIZE - 1)
            {
                g_cmd_buffer[g_cmd_index++] = (char)byte;
            }
            else
            {
                // 缓冲区满，丢弃当前命令并重新开始
                g_cmd_index = 0;
            }
        }
        // 忽略其他控制字符
    }
}

/**
 * @brief 获取当前运动状态
 */
Motion_Mode_t BT_GetMotionMode(void)
{
    return g_motion_mode;
}

/**
 * @brief 获取当前运动速度
 */
uint8_t BT_GetMotionSpeed(void)
{
    return g_motion_speed;
}

/**
 * @brief 获取完整的蓝牙状态
 */
const BT_State_t* BT_GetState(void)
{
    static BT_State_t state;
    state.motion_mode = g_motion_mode;
    state.motion_speed = g_motion_speed;
    state.base_speed = g_base_speed;
    state.vacuum_enabled = g_vacuum_enabled;
    state.vacuum_speed = g_vacuum_speed;
    state.key_control_mode = g_key_control_mode;
    state.emergency_stop = g_emergency_stop;
    return &state;
}

/**
 * @brief 直接设置吸盘转速（用于硬件K1同步状态）
 */
void BT_SetVacuumSpeedDirect(uint8_t speed_percent)
{
    if(speed_percent > 100) speed_percent = 100;
    g_vacuum_speed = speed_percent;
    g_vacuum_enabled = (speed_percent > 0) ? 1 : 0;
}

/**
 * @brief 发送数据包（保留用于兼容）
 */
void BT_SendPacket(const BT_Packet_t *packet)
{
    // 保留接口，但现在使用字符串方式
    char buf[32];
    sprintf(buf, "PKT:%02X,%02X\r\n", packet->cmd, packet->data);
    BT_SendResponse(buf);
}
