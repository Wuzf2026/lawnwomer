#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerRequest

def query_lawnmower_status():
    rospy.init_node('lawnmower_status_query_client', anonymous=True)
    
    # 等待服务启动
    rospy.wait_for_service('/lawnmower/status_query')
    try:
        # 创建服务客户端
        status_client = rospy.ServiceProxy('/lawnmower/status_query', Trigger)
        req = TriggerRequest()
        resp = status_client(req)
        
        # 打印状态信息
        rospy.loginfo("\n===== Lawnmower Status =====")
        rospy.loginfo(f"Success: {resp.success}")
        rospy.loginfo(f"Message:\n{resp.message}")
        rospy.loginfo("============================")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    query_lawnmower_status()