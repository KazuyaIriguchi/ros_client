import streamlit as st
import rospy
import threading
import rosservice
from std_msgs.msg import String

def get_rosparams():
    return rospy.get_param_names()

def get_rostopics():
    return rospy.get_published_topics()

def get_rosservices():
    # Use rosservice module to get a list of services
    return rosservice.get_service_list()

threading.Thread(target=lambda: rospy.init_node('streamlit_ros_client', disable_signals=True)).start()

def main():
    st.title('ROS Client Web UI')

    # ROS Parameters
    st.subheader("ROS Parameters")
    params = get_rosparams()
    selected_param = st.selectbox("Select Parameter", params)
    if selected_param:
        st.write(f"Value: {rospy.get_param(selected_param)}")

    # ROS Topics
    st.subheader("ROS Topics")
    topics = [t[0] for t in get_rostopics()]
    selected_topic = st.selectbox("Select Topic for Publishing (std_msgs/String)", topics)
    if selected_topic:
        message = st.text_input("Enter message to publish:")
        if st.button("Publish to Topic"):
            publisher = rospy.Publisher(selected_topic, String, queue_size=10)
            publisher.publish(message)
            st.success("Message published.")

    # ROS Services
    st.subheader("ROS Services")
    services = get_rosservices()
    st.write(services)

if __name__ == '__main__':
    main()
