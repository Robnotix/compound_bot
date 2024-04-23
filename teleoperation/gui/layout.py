from agv_control.layout import layout as agv_control_layout
from robot_arm_control.layout import layout as robot_arm_control_layout
from cameras.layout import layout as cameras_layout
# from config.layout import layout as config_layout
from lidar.layout import layout as lidar_layout
from point_clouds.layout import layout as point_clouds_layout

from dash import html, dcc
import dash_bootstrap_components as dbc

layout = dbc.Container(
    [
        html.H2("Teleoperation"),
        dbc.Row(
            [
                dbc.Col(
                    cameras_layout, width=4),
                dbc.Col(
                    [
                        lidar_layout,
                        point_clouds_layout
                    ],
                    width=4,
                ),
                dbc.Col(
                    [
                        agv_control_layout,
                        robot_arm_control_layout
                    ], 
                    width=4),
                
            ],
            
        )
    ]
)
