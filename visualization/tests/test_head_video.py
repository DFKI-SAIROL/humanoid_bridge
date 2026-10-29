"""Head video wire compatibility, RGB colour order, and display load limits."""
import struct
import unittest
from types import SimpleNamespace as S
from unittest.mock import patch
import numpy as np
from h1_2_viewer.native_video import video_payload
from h1_2_viewer.head_camera_ros import decode_rgb


def firmware_packet(payload, height=360, endian='<'):
    data = (b'\x00\x01\x00\x00' if endian == '<' else b'\x00\x00\x00\x00')
    data += struct.pack(endian+'QII', 1234, height, len(payload)) + payload
    return data + b'\x00'*((-len(data))%4)


class HeadVideo(unittest.TestCase):
    def test_observed_firmware_cdr_both_endiannesses_and_resolution_filter(self):
        payload=b'\x00\x00\x00\x01\x67abc'
        for endian in ('<', '>'):
            packet=firmware_packet(payload, endian=endian)
            self.assertEqual(video_payload(packet), payload)
            self.assertIsNone(video_payload(packet,720))
        with self.assertRaises(ValueError):video_payload(packet[:-5])

    def test_public_three_array_schema_and_padding(self):
        data=b'\x00\x01\x00\x00'+struct.pack('<Q', 1234)
        for payload in (b'720', b'360-payload', b''):
            data+=b'\x00'*((-len(data))%4)
            data+=struct.pack('<I',len(payload))+payload
        self.assertEqual(video_payload(data),b'360-payload')
        self.assertEqual(video_payload(data,720),b'720')
        self.assertIsNone(video_payload(data,180))

    def test_standard_rgb_padding_and_colour_order(self):
        msg=S(width=2,height=1,encoding='bgr8',step=8,data=bytes([0,0,255,0,255,0,99,99]))
        np.testing.assert_array_equal(decode_rgb(msg), [[[255,0,0],[0,255,0]]])
        msg.data=b'\x00'
        with self.assertRaises(ValueError):decode_rgb(msg)

    def test_head_image_is_rate_limited_and_jpeg_encoded(self):
        from h1_2_viewer.configuration import load_profile
        from h1_2_viewer.renderer import Dashboard
        model=S(log_geometry=lambda:None,hand_reference_note='reference')
        with patch('rerun.log') as log:
            d=Dashboard(model,load_profile());log.reset_mock()
            for i in range(100):d.head_rgb(np.zeros((36,64,3),dtype=np.uint8),elapsed=i/100)
            images=[c for c in log.call_args_list if c.args[0]=='cameras/head/rgb']
            self.assertEqual(len(images),5)
            self.assertEqual(d.counts['head_rgb'],100)
            self.assertEqual(images[0].args[1].__class__.__name__,'EncodedImage')

    def test_h264_decode_and_output_size(self):
        try:import av
        except ImportError:self.skipTest('optional video extra not installed')
        from h1_2_viewer.native_video import FrontVideoDecoder
        encoder=av.CodecContext.create('libx264','w')
        encoder.width,encoder.height,encoder.pix_fmt=64,36,'yuv420p'
        encoder.options={'preset':'ultrafast','tune':'zerolatency'}
        decoder=FrontVideoDecoder(max_width=32)
        images=[]
        with patch('h1_2_viewer.native_video.time.monotonic',side_effect=range(100)):
            packets=[]
            for i in range(5):
                frame=av.VideoFrame.from_ndarray(np.full((36,64,3),i*20,dtype=np.uint8),format='rgb24')
                packets.extend(encoder.encode(frame))
            packets.extend(encoder.encode(None))
            for packet in packets:
                image=decoder.receive(firmware_packet(bytes(packet)))
                if image is not None:images.append(image)
        self.assertTrue(images)
        self.assertEqual(images[-1].shape,(18,32,3))
        self.assertEqual(images[-1].dtype,np.uint8)
