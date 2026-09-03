#include "vofa.h"

/* 收到指令后回显原始数据包，用于排查 VOFA 下行链路；调试完置 0 */
#define DEBUG_VOFA_RX   1

/* VOFA指令解析：slider调参 + button运动控制 */
void ParseVOFACommand(void)
{
    if (vofa_rx_flag == 1)
    {
        char buf[100];
        strncpy(buf, vofa_rx_packet, 99);
        vofa_rx_flag = 0;

#if DEBUG_VOFA_RX
        printf("RX:[%s]\n", buf);   /* 回显收到的完整指令 */
#endif

        char *Tag = strtok(buf, ",");

        if (Tag != NULL && strcmp(Tag, "slider") == 0)
        {
            char *Name = strtok(NULL, ",");
            char *Value = strtok(NULL, ",");

            if (Name != NULL && Value != NULL)
            {
                if (strcmp(Name, "Speed") == 0)
                {
                    BaseSpeed = atoi(Value);
#if DEBUG_VOFA_RX
                    printf("->BaseSpeed=%u\n", BaseSpeed);
#endif
                }
                else if (strcmp(Name, "SpeedKp") == 0)
                    SpeedPID_L.Kp = SpeedPID_R.Kp = atof(Value);
                else if (strcmp(Name, "SpeedKi") == 0)
                    SpeedPID_L.Ki = SpeedPID_R.Ki = atof(Value);
                else if (strcmp(Name, "SpeedKd") == 0)
                    SpeedPID_L.Kd = SpeedPID_R.Kd = atof(Value);
                else if (strcmp(Name, "SpeedFF") == 0)
                    SpeedPID_L.FeedForward = SpeedPID_R.FeedForward = atof(Value);
                else if (strcmp(Name, "SpeedFF2") == 0)
                    SpeedPID_L.FeedForward2 = SpeedPID_R.FeedForward2 = atof(Value);
                else if (strcmp(Name, "SpeedDZ") == 0)
                    SpeedPID_L.DeadZone = SpeedPID_R.DeadZone = atof(Value);
                else if (strcmp(Name, "SpeedSep") == 0)
                    SpeedPID_L.IntSepThresh = SpeedPID_R.IntSepThresh = atof(Value);
                else if (strcmp(Name, "PosKp") == 0)
                    PosPID.Kp = atof(Value);
                else if (strcmp(Name, "PosKi") == 0)
                    PosPID.Ki = atof(Value);
                else if (strcmp(Name, "PosKd") == 0)
                    PosPID.Kd = atof(Value);
                else if (strcmp(Name, "TrackSpeed") == 0)
                    TrackSpeed = atof(Value);
            }
        }
        else if (Tag != NULL && strcmp(Tag, "button") == 0)
        {
            char *Action = strtok(NULL, ",");
            char *State  = strtok(NULL, ",");

            if (Action != NULL && State != NULL)
            {
                int state = atoi(State);

                if (state == 1)
                {
                    if (strcmp(Action, "forward") == 0)
                        CarCommand = CAR_CMD_FORWARD;
                    else if (strcmp(Action, "back") == 0)
                        CarCommand = CAR_CMD_BACK;
                    else if (strcmp(Action, "left") == 0)
                        CarCommand = CAR_CMD_LEFT;
                    else if (strcmp(Action, "right") == 0)
                        CarCommand = CAR_CMD_RIGHT;
                    else if (strcmp(Action, "track") == 0)
                    {
                        /* 任务3/6是纯STM32球杆动作, 小车不进TRACK */
                        if (CarMode == CAR_MODE_VOFA && TASK_IS_CAR(TaskID))
                        {
                            CarMode = CAR_MODE_TRACK;
                            CarCommand = CAR_CMD_STOP;
                            PID_Init(&PosPID);
                        }
                        else
                        {
                            CarMode = CAR_MODE_VOFA;
                            CarCommand = CAR_CMD_STOP;
                        }
                    }
                }
                else
                {
                    if (strcmp(Action, "track") != 0)
                        CarCommand = CAR_CMD_STOP;
                }
#if DEBUG_VOFA_RX
                printf("->Cmd=%d Mode=%d\n", (int)CarCommand, (int)CarMode);
#endif
            }
        }
    }
}
