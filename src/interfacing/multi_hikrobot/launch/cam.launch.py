import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from dataclasses import dataclass


@dataclass
class CameraConfig:
    guid: str
    frame_id: str
    stream_name: str


def generate_launch_description():
    configs = [
        CameraConfig(
            guid='Hikrobot-MV-CU013-80GC-DA8386154',
            frame_id='camera_pano_ne',
            stream_name='pano_ne_raw'
        ),

        CameraConfig(
            guid='Hikrobot-MV-CU013-80GC-DA8386084',
            frame_id='camera_pano_ee',
            stream_name='pano_ee_raw'
        ),
        CameraConfig(
            guid='Hikrobot-MV-CU013-80GC-DA8386152',
            frame_id='camera_pano_se',
            stream_name='pano_se_raw'
        ),
        CameraConfig(
            guid='Hikrobot-MV-CU013-80GC-DA8386134',
            frame_id='camera_pano_ss',
            stream_name='pano_ss_raw'
        ),
        CameraConfig(
            guid='Hikrobot-MV-CU013-80GC-DA8386093',
            frame_id='camera_pano_sw',
            stream_name='pano_sw_raw'
        ),
        CameraConfig(
            guid='Hikrobot-MV-CU013-80GC-DA8386110',
            frame_id='camera_pano_ww',
            stream_name='pano_ww_raw'
        ),
        CameraConfig(
            guid='Hikrobot-MV-CU013-80GC-DA8386109',
            frame_id='camera_pano_nw',
            stream_name='pano_nw_raw'
        ),
        CameraConfig(
            guid='Hikrobot-MV-CU013-80GC-DA8386098',
            frame_id='camera_pano_nn',
            stream_name='pano_nn_raw'
        ),
    ]

    nodes = []

    for config in configs:
        node = Node(
            name=f'camera_driver_{config.frame_id}',
            package='camera_aravis2',
            executable='camera_driver_gv',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'guid': config.guid,
                    'frame_id': config.frame_id,
                    'stream_names': [config.stream_name],
                    'camera_info_urls': [os.path.join(
                        get_package_share_directory('multi_hikrobot'),
                        f'config/{config.frame_id}.yaml')],
                    'verbose': False,
                    'DeviceControl': {},
                    'TransportLayerControl': {
                        'GevSCPSPacketSize': 9156
                    },
                    'ImageFormatControl': {},
                    'AcquisitionControl': {
                        # 'ExposureTime': 8000.0,
                        # 'ExposureAuto': 'Off',
                        'AcquisitionFrameRateEnable': True,
                        'AcquisitionFrameRate': 30.0
                    },
                }
            ]
        )
        nodes.append(node)

    return LaunchDescription(nodes)
