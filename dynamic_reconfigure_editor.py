import ast
import sys
import packaging.version
import streamlit as st

if packaging.version.parse(st.__version__) <= packaging.version.parse('1.10.0'):
    import streamlit.cli as stcli
else:
    import streamlit.web.cli as stcli
from dynamic_reconfigure import find_reconfigure_services
from dynamic_reconfigure import encoding as dre
from dynamic_reconfigure import client as drc
import rospy
import math
from typing import Any, Dict, Optional, List


def get_value(client: drc.Client, param_name: str) -> Any:
    config: Dict[str, Any] = client.get_configuration()
    return config[param_name]


def update_one(client: drc.Client, key: str, param_name: str) -> None:
    state = st.session_state[key]
    value = state
    if isinstance(state, tuple):
        value = state[1]

    client.update_configuration({param_name: value})
    create_client_and_display(client, client.name)


def render_slider(client: drc.Client, param: Dict[str, Any]) -> None:

    param_name = param['name']
    value = get_value(client, param_name)
    min_value = param['min']
    max_value = param['max']
    step = (max_value - min_value) / 10.0
    if isinstance(min_value, int):
        step = math.ceil(step)
    key = f"slider_{param_name.replace('/','_')}"
    col_1, col_2 = st.columns(2)
    with col_1:
        st.slider(label=param_name,
                  min_value=min_value,
                  max_value=max_value,
                  step=step,
                  value=value,
                  key=key,
                  on_change=update_one,
                  args=(
                      client,
                      key,
                      param_name,
                  ))
    with col_2:
        key = f"num_input_{param_name.replace('/','_')}"

        st.number_input(label=param_name,
                        min_value=min_value,
                        max_value=max_value,
                        value=value,
                        step=step,
                        key=key,
                        help=param['description'],
                        on_change=update_one,
                        args=(
                            client,
                            key,
                            param_name,
                        ))


def render_options(client: drc.Client, param: Dict[str, Any]) -> None:

    enum_dict_as_str = param['edit_method']
    enum_dict = ast.literal_eval(enum_dict_as_str)
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

    key = f"options_{param_name.replace('/','_')}"
    st.selectbox(label=param_name,
                 options=names,
                 index=value_idx,
                 key=key,
                 help=param['description'],
                 on_change=update_one,
                 args=(
                     client,
                     key,
                     param_name,
                 ))


def render_checkbox(client: drc.Client, param: Dict[str, Any]) -> None:
    param_name = param['name']
    key = f"checkbox_{param_name.replace('/','_')}"
    st.checkbox(label=param_name,
                value=get_value(client, param_name),
                help=param['description'],
                key=key,
                on_change=update_one,
                args=(
                    client,
                    key,
                    param_name,
                ))


def render_text_input(client: drc.Client, param: Dict[str, Any]) -> None:
    param_name = param['name']
    key = f"txt_input_{param_name.replace('/','_')}"
    st.text_input(label=param_name,
                  value=get_value(client, param_name),
                  help=param['description'],
                  key=key,
                  on_change=update_one,
                  args=(
                      client,
                      key,
                      param_name,
                  ))


def create_client(server: str) -> drc.Client:
    return drc.Client(server, timeout=2.0)


def get_or_create_client(client: Optional[drc.Client], server: str) -> drc.Client:
    if client is None:
        return create_client(server)

    if client.name == server:
        return client

    client.close()
    return create_client(server)


def render_parameters(client: drc.Client, params_list: List[Dict[str, Any]]):

    for _, param in enumerate(params_list):
        if param['edit_method'] != '':
            render_options(client, param)
        elif param['type'] in ('int', 'double'):
            render_slider(client, param)
        elif param['type'] == 'bool':
            render_checkbox(client, param)
        else:
            render_text_input(client, param)


def render_for_groups(client: drc.Client, group: dre.Config) -> None:
    with st.expander(f"Group {group['name']}", expanded=True):
        params = group.get('parameters')
        render_parameters(client, params)


def create_client_and_display(curr_client: drc.Client, server: str) -> None:
    st.markdown(f"### Configuration for `{server}` ⚙️")
    client = get_or_create_client(curr_client, server)
    desc: dre.Config = client.get_group_descriptions(timeout=1.0)

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
    render_parameters(client, params_list)

    groups: dre.Config = desc.get('groups')
    if groups:
        for group in groups.values():
            render_for_groups(client, group)


def refresh_servers(container) -> None:
    servers = find_reconfigure_services()
    for server in servers:
        key = f"button_{server.replace('/','_')}"
        container.button(label=server, on_click=create_client_and_display, args=(
            None,
            server,
        ), key=key)


def main() -> None:
    if packaging.version.parse(st.__version__) <= packaging.version.parse('1.12.0'):
        st.markdown("""
        <style>
        [data-testid="stSidebar"][aria-expanded="true"] > div:first-child {
        width: 500px;
        }
        [data-testid="stSidebar"][aria-expanded="false"] > div:first-child {
        width: 500px;
        margin-left: -500px;
        }
        </style>
        """,
                    unsafe_allow_html=True)
    st.sidebar.markdown("# Dynamic reconfigure editor 💡")
    container = st.sidebar.container()
    container.subheader("List of available servers")
    refresh_servers(container)


if __name__ == '__main__':
    rospy.init_node('streamlit_ros_node', anonymous=True)

    if st._is_running_with_streamlit:
        main()
    else:
        sys.argv = [
            "streamlit", "run", sys.argv[0], "--server.headless", "true", "--browser.gatherUsageStats", "false",
            "--server.port", "8050"
        ]
        sys.exit(stcli.main())
