import ast
import sys
import streamlit as st
import streamlit.web.cli as stcli
from dynamic_reconfigure import find_reconfigure_services
from dynamic_reconfigure import client as drc
import rospy
import math
from typing import Any, Dict, List
import time


def get_value(client: drc.Client, param_name):
    config = client.get_configuration()
    return config[param_name]


def render_slider(client: drc.Client, param: Dict[str, Any]):

    def update_one(param_name: str):
        value = st.session_state[f"slider_{param_name}"]
        new_config = client.update_configuration({param_name: value})
        create_client_and_display(client.name)

    param_type = param['type']
    param_name = param['name']
    value = get_value(client, param_name)
    min_value = param['min']
    max_value = param['max']
    step = (max_value - min_value) / 10.0
    if isinstance(min_value, int):
        step = math.ceil(step)
    return st.slider(label=param_name,
                     min_value=min_value,
                     max_value=max_value,
                     step=step,
                     value=value,
                     key=f"slider_{param_name}",
                     on_change=update_one,
                     args=(param_name, ))


def render_options(client: drc.Client, param):

    def update_one(param_name: str):
        name, value = st.session_state[f"options_{param_name}"]
        # st.write(f"selected {value}")
        new_config = client.update_configuration({param_name: value})
        create_client_and_display(client.name)

    enum_dict_as_str = param['edit_method']
    enum_dict = ast.literal_eval(enum_dict_as_str)
    # rospy.logwarn(param)
    param_name = param['name']

    names = []
    idx = 0
    value_idx = idx
    current_value = get_value(client, param_name=param_name)
    for d in enum_dict['enum']:
        names.append((d['name'], d['value']))
        if d['value'] == current_value:
            value_idx = idx
        idx += 1
    option = st.selectbox(label=param_name,
                          options=names,
                          index=value_idx,
                          key=f"options_{param_name}",
                          on_change=update_one,
                          args=(param_name, ))


def render_checkbox(client: drc.Client, param):
    param_name = param['name']

    def update_one():
        value = st.session_state[f"options_{param_name}"]
        new_config = client.update_configuration({param_name: value})

    st.checkbox(label=param_name,
                value=get_value(client, param_name),
                help=param['description'],
                on_change=update_one,
                key=f"options_{param_name}")


def create_client_and_display(server):
    st.markdown(f"### Configuration for `{server}` ⚙️")
    client = drc.Client(server, timeout=2.0)
    desc = client.get_group_descriptions(timeout=1.0)
    failure_msg = f"failure in client {client.name}"
    if desc is None:
        st.warning(failure_msg, icon="⚠️")
        rospy.logerr(failure_msg)
        return
    params_list = desc.get('parameters')
    if params_list is None:
        st.warning(failure_msg, icon="⚠️")
        rospy.logerr(failure_msg)
        return

    for _, param in enumerate(params_list):
        if param['edit_method'] != '':
            render_options(client, param)
        elif param['type'] in ('int', 'double'):
            render_slider(client, param)
        elif param['type'] == 'bool':
            render_checkbox(client, param)
        else:
            st.markdown(f"#### `NOT IMPLEMENTED` 🛠")
            st.write(param)


def refresh_servers(container, should_empty: bool) -> None:
    servers = find_reconfigure_services()
    # if should_empty:
    #     placeholder.empty()
    for server in servers:
        key = f"button_{server.replace('/','_')}"
        # if key in st.session_state:
        #     del st.session_state[key]
        #     time.sleep(0.2)
        container.button(label=server, on_click=create_client_and_display, args=(server, ), key=key)


def create_button():

    def adjust_value():
        value = st.session_state["test_button"]
        st.write(f"You pressed, current state {value}")

    def read_slider():
        value = st.session_state["test_slider"]
        st.write(f"Slider, current value {value}")

    st.slider(label="param_name", min_value=0, max_value=10, step=1, value=5, key=f"test_slider", on_change=read_slider)
    st.button(label="Adjust value", on_click=adjust_value, key="test_button")


def main():
    # placeholder = container.empty()
    # title_container = placeholder.container()
    # other_container = placeholder.container()
    # st.sidebar.button("Refresh servers", on_click=refresh_servers, args=(
    #     placeholder,
    #     title_container,
    #     True,
    # ))
    # title_container.text_input("Search node")
    # create_button()

    st.sidebar.markdown("# Dynamic reconfigure editor 💡")
    container = st.sidebar.container()
    container.subheader("List of available servers")
    refresh_servers(container, False)


if __name__ == '__main__':
    rospy.init_node('streamlit_ros_node', anonymous=True)
    # st.set_option("browser.gatherUsageStats", False)
    # st.set_option("server.port", 8050)
    # st.set_option("server.headless", True)
    if st._is_running_with_streamlit:
        main()
    else:
        sys.argv = ["streamlit", "run", sys.argv[0], "--server.headless", "true", "--browser.gatherUsageStats", "false"]
        sys.exit(stcli.main())
