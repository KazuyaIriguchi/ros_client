import streamlit as st
import subprocess

def run_command(command):
    """Run a shell command and return the output."""
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
    output, error = process.communicate()
    return output.decode("utf-8")

# Streamlit UI
st.title("ROS Client Tool")

# ROS Parameters
st.subheader("Parameters")
param_name = st.text_input("Enter Parameter Name")
if st.button("Get Parameter"):
    # Get and display the selected parameter here
    command = f"rosparam get {param_name}"
    output = run_command(command)
    st.write(f"Parameter Value: {output}")

# ROS Topics
st.subheader("Topics")
if st.button("List Topics"):
    # List ROS topics
    command = "rostopic list"
    output = run_command(command)
    st.write("Topics:")
    st.write(output)

# ROS Services
st.subheader("Services")
if st.button("List Services"):
    # List ROS services
    command = "rosservice list"
    output = run_command(command)
    st.write("Services:")
    st.write(output)
