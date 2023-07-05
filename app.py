import streamlit as st
import subprocess

def run_command(command):
    """Run a shell command and return the output."""
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
    output, error = process.communicate()
    return output.decode("utf-8")

def get_rosparams():
    """Get list of ROS parameters"""
    output = run_command("rosparam list")
    return output.splitlines()

def get_rostopics():
    """Get list of ROS topics"""
    output = run_command("rostopic list")
    return output.splitlines()

def get_rosservices():
    """Get list of ROS services"""
    output = run_command("rosservice list")
    return output.splitlines()

# Streamlit UI
st.title("ROS Client Tool")

# ROS Parameters
st.subheader("Parameters")
params = get_rosparams()
selected_param = st.selectbox("Select Parameter", params)
if st.button("Get Parameter Value"):
    # Get and display the selected parameter value
    command = f"rosparam get {selected_param}"
    output = run_command(command)
    st.write(f"Parameter Value: {output}")

# ROS Topics
st.subheader("Topics")
topics = get_rostopics()
selected_topic = st.selectbox("Select Topic", topics)
if st.button("Publish to Topic"):
    # Publish to the selected topic here (you'll need to implement message publishing based on the topic type)
    pass

# ROS Services
st.subheader("Services")
services = get_rosservices()
selected_service = st.selectbox("Select Service", services)
if st.button("Call Service"):
    # Call the selected service here (you'll need to implement service calling based on the service type)
    pass
