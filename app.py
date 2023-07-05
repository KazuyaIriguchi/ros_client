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

def import_service_type(service_type):
    pkg, srv = service_type.split('/')
    module = importlib.import_module(pkg + '.srv')
    return getattr(module, srv)

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
    selected_service = st.selectbox("Select Service for Calling", services)
    if selected_service:
        service_type = rosservice.get_service_type(selected_service)
        SrvClass = import_service_type(service_type)

        st.text(f"Service Type: {service_type}")

        # Parse the service arguments
        srv_args = SrvClass._request_class.__slots__
        srv_args_dict = {}

        st.text("Enter the service arguments:")
        for arg in srv_args:
            srv_args_dict[arg] = st.text_input(arg)

        if st.button("Call Service"):
            service_proxy = rospy.ServiceProxy(selected_service, SrvClass)

            try:
                response = service_proxy(**srv_args_dict)
                st.write(f"Service Response: {response}")

            except rospy.ServiceException as e:
                st.error(f"Service call failed: {e}")


if __name__ == '__main__':
    main()
