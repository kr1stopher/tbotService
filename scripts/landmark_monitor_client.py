# from __future__ import print_function
import rospy
import tbot_msgs.srv

def main():
    #initialize node
    rospy.init_node('landmark_client')

    #Service definitions used as container for requets and response type, must be used whenever creating or calling a service
    get_closest = rospy.ServiceProxy('get_closest',tbot_msgs.srv.GetClosest)
    get_distance = rospy.ServiceProxy('get_distance', tbot_msgs.srv.GetDistance)

    #check to see if the service is running, wait 5 seconds
    try:
        rospy.wait_for_service('get_closest', timeout =5)
        rospy.wait_for_service('get_distance', timeout = 5)
    except rospy.ROSException:
        rospy.logerr('Services did not start in less than 5 seconds')
        return

    #create a request
    req = tbot_msgs.srv.GetClosestRequest()
    #call get closest using request
    resp = get_closest(req)
    print('Closest: {}'.format(resp.name))


    landmarks = ['Dumpster', 'Barrier', 'Cylinder', 'Bookshelf', 'Cube']
    #for each landmark, create a distance request, send request, and print results 
    for landmark in landmarks:
        req = tbot_msgs.srv.GetDistanceRequest()
        req.name = landmark
        resp = get_distance(req)
        print('{}: {}'.format(landmark,resp.distance))

if __name__ == '__main__':
    main()
