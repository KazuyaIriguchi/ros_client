import streamlit as st
import rospy
import threading
import rosservice
import importlib
import time


def get_rosparams():
    return rospy.get_param_names()

def get_rostopics():
    return rospy.get_published_topics()

def get_rosservices():
    # Use rosservice module to get a list of services
    return rosservice.get_service_list()

def import_message_type(message_type):
    pkg, msg = message_type.split('/')
    module = importlib.import_module(pkg + '.msg')
    return getattr(module, msg)

def callback(msg, callback_container):
    callback_container.append(msg)

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
    topics = {t[0]: t[1] for t in get_rostopics()}
    selected_topic = st.selectbox("Select Topic for Subscribing", list(topics.keys()))
    if selected_topic:
        if st.button("Receive from Topic"):
            message_type = topics[selected_topic]
            MsgClass = import_message_type(message_type)

            # Container to store messages received by callback
            callback_container = []

            subscriber = rospy.Subscriber(selected_topic, MsgClass, callback, callback_container)
            st.text("Waiting for messages...")

            timeout = 5.0  # 5 seconds timeout
            start_time = time.time()

            # Wait until a message is received or timeout
            while not callback_container and time.time() - start_time < timeout:
                pass

            subscriber.unregister()

            if callback_container:
                for msg in callback_container:
                    st.write(f"Received message: {msg}")
            else:
                st.warning("No message received within the timeout period.")

    # ROS Services
    st.subheader("ROS Services")
    services = get_rosservices()
    st.write(services)


if __name__ == '__main__':
    main()
