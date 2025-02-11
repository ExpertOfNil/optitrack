use bytes::{Buf, BufMut, BytesMut};
use glam::{Quat, Vec3};
use std::{
    error,
    io::{self, BufRead},
};

pub trait Encoder<Item> {
    type Error: From<io::Error>;
    fn encode(&mut self, item: Item, dst: &mut BytesMut) -> Result<(), Self::Error>;
}

pub trait Decoder {
    type Item;
    type Error: From<io::Error>;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error>;
}

#[derive(Debug)]
pub enum Message {
    PingResponse,
    FrameData(Box<FrameData>),
    ModelDef(Box<ModelDef>),
    Unknown,
}

impl Message {
    pub fn from_bytes(mut src: BytesMut) -> Result<Self, Box<dyn std::error::Error>> {
        if src.len() < size_of::<u16>() {
            return Err(format!(
                "Not enough bytes for message ID.  Expected: {}, Got: {}",
                src.len(),
                size_of::<u16>()
            )
            .into());
        }
        let message_id = src.get_u16_le();
        log::debug!("Message ID: {}", message_id);
        let message_id = match message_id.into() {
            MessageId::PingResponse => Message::PingResponse,
            MessageId::FrameData => {
                let mut codec = FrameDataCodec;
                let frame_data = codec.decode(&mut src)?;
                Message::FrameData(Box::new(frame_data))
            }
            MessageId::ModelDef => {
                let mut codec = ModelDefCodec;
                let modeldef = codec.decode(&mut src)?;
                Message::ModelDef(Box::new(modeldef))
            }
            id => {
                log::error!("Got message type: {:?}", id);
                unimplemented!()
            }
        };
        Ok(message_id)
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u16)]
pub enum MessageId {
    Ping = 0,
    PingResponse = 1,
    Request = 2,
    Response = 3,
    RequestModelDef = 4,
    ModelDef = 5,
    RequestFrameData = 6,
    FrameData = 7,
    MessageString = 8,
    Disconnect = 9,
    KeepAlive = 10,
    DisconnectByTimeout = 11,
    EchoRequest = 12,
    EchoResponse = 13,
    Discovery = 14,
    Unrecognized = 100,
}

impl From<&[u8; 2]> for MessageId {
    fn from(value: &[u8; 2]) -> Self {
        let value: u16 = *bytemuck::from_bytes(value);
        value.into()
    }
}

impl From<u16> for MessageId {
    fn from(value: u16) -> Self {
        match value {
            0 => Self::Ping,
            1 => Self::PingResponse,
            2 => Self::Request,
            3 => Self::Response,
            4 => Self::RequestModelDef,
            5 => Self::ModelDef,
            6 => Self::RequestFrameData,
            7 => Self::FrameData,
            8 => Self::MessageString,
            9 => Self::Disconnect,
            10 => Self::KeepAlive,
            11 => Self::DisconnectByTimeout,
            12 => Self::EchoRequest,
            13 => Self::EchoResponse,
            14 => Self::Discovery,
            _ => Self::Unrecognized,
        }
    }
}

#[derive(Debug, Default)]
pub struct FrameDataCodec;

impl Encoder<FrameData> for FrameDataCodec {
    type Error = Box<dyn std::error::Error>;
    fn encode(&mut self, item: FrameData, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least message id, packet size, frame number, all counts,
        // timecodes, timestamps, and frame parameters
        //dst.reserve(78);
        dst.extend_from_slice(&item.packet_size.to_le_bytes()[..]);
        dst.extend_from_slice(&item.frame_number.to_le_bytes()[..]);
        dst.extend_from_slice(&item.markerset_count.to_le_bytes()[..]);
        let mut markerset_codec = MarkerSetCodec::default();
        for ms in item.markersets.into_iter() {
            markerset_codec.encode(ms, dst)?;
        }
        dst.extend_from_slice(&item.unlabeled_marker_count.to_le_bytes()[..]);
        for pos in item.unlabeled_marker_positions.into_iter() {
            dst.extend_from_slice(&pos.x.to_le_bytes()[..]);
            dst.extend_from_slice(&pos.y.to_le_bytes()[..]);
            dst.extend_from_slice(&pos.z.to_le_bytes()[..]);
        }
        dst.extend_from_slice(&item.rigid_body_count.to_le_bytes()[..]);
        let mut rigid_body_codec = RigidBodyCodec::default();
        for rb in item.rigid_bodies.into_iter() {
            rigid_body_codec.encode(rb, dst)?;
        }
        dst.extend_from_slice(&item.skeleton_count.to_le_bytes()[..]);
        let mut skeleton_codec = SkeletonCodec::default();
        for skeleton in item.skeletons.into_iter() {
            skeleton_codec.encode(skeleton, dst)?;
        }
        dst.extend_from_slice(&item.labeled_marker_count.to_le_bytes()[..]);
        let mut labeled_marker_codec = LabeledMarkerCodec::default();
        for lmp in item.labeled_marker_positions.into_iter() {
            labeled_marker_codec.encode(lmp, dst)?;
        }
        dst.extend_from_slice(&item.force_plate_count.to_le_bytes()[..]);
        let mut force_plate_codec = ForcePlateCodec::default();
        for fp in item.force_plates.into_iter() {
            force_plate_codec.encode(fp, dst)?;
        }
        dst.extend_from_slice(&item.device_count.to_le_bytes()[..]);
        let mut device_codec = DeviceCodec::default();
        for device in item.devices.into_iter() {
            device_codec.encode(device, dst)?;
        }
        dst.extend_from_slice(&item.timecode.to_le_bytes()[..]);
        dst.extend_from_slice(&item.timecode_sub.to_le_bytes()[..]);
        let mut stamps_codec = StampsCodec::default();
        stamps_codec.encode(item.stamps, dst)?;
        let mut frame_parameters_codec = FrameParametersCodec::default();
        frame_parameters_codec.encode(item.frame_parameters, dst)?;
        Ok(())
    }
}

impl Decoder for FrameDataCodec {
    type Error = Box<dyn error::Error>;
    type Item = FrameData;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        let packet_size = src.get_u16_le();
        log::debug!("Packet Size: {} bytes", packet_size);
        let frame_number = src.get_u32_le();
        log::debug!("Frame #: {}", frame_number);
        let markerset_count = src.get_u32_le();
        log::debug!("MarkerSet Count: {}", markerset_count);
        let markerset_bytes = src.get_u32_le();
        log::debug!("MarkerSet Bytes: {}", markerset_bytes);
        let mut markerset_codec = MarkerSetCodec::default();
        let markersets: Vec<MarkerSet> = (0..markerset_count)
            .map(|_| markerset_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        log::debug!("MarkerSets: {:?}", markersets);
        let unlabeled_marker_count = src.get_u32_le();
        log::debug!("Unlabeled Marker Count: {}", unlabeled_marker_count);
        let unlabeled_marker_bytes = src.get_u32_le();
        log::debug!("Unlabeled Marker Bytes: {}", unlabeled_marker_bytes);
        let unlabeled_marker_positions: Vec<Vec3> = (0..unlabeled_marker_count)
            .map(|_| Vec3 {
                x: src.get_f32_le(),
                y: src.get_f32_le(),
                z: src.get_f32_le(),
            })
            .collect();
        log::debug!(
            "Unlabeled Marker Positions: {:?}",
            unlabeled_marker_positions
        );
        let rigid_body_count = src.get_u32_le();
        log::debug!("RigidBody Count: {}", rigid_body_count);
        let rigid_body_bytes = src.get_u32_le();
        log::debug!("RigidBody Bytes: {}", rigid_body_bytes);
        let mut rigid_body_codec = RigidBodyCodec::default();
        let rigid_bodies: Vec<RigidBody> = (0..rigid_body_count)
            .map(|_| rigid_body_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        log::debug!("RigidBodies: {:?}", rigid_bodies);
        let skeleton_count = src.get_u32_le();
        log::debug!("Skeleton Count: {}", skeleton_count);
        let skeleton_bytes = src.get_u32_le();
        log::debug!("Skeleton Bytes: {}", skeleton_bytes);
        let mut skeleton_codec = SkeletonCodec::default();
        let skeletons: Vec<Skeleton> = (0..skeleton_count)
            .map(|_| skeleton_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        log::debug!("Skeletons: {:?}", skeletons);
        let asset_count = src.get_u32_le();
        log::debug!("Asset Count: {}", asset_count);
        let asset_bytes = src.get_u32_le();
        log::debug!("Asset Bytes: {}", asset_bytes);
        let mut asset_codec = AssetCodec::default();
        let assets: Vec<Asset> = (0..asset_count)
            .map(|_| asset_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        log::debug!("Assets: {:?}", assets);
        let labeled_marker_count = src.get_u32_le();
        log::debug!("Labeled Marker Count: {}", labeled_marker_count);
        let labeled_marker_bytes = src.get_u32_le();
        log::debug!("Labeled Marker Bytes: {}", labeled_marker_bytes);
        let mut labeled_marker_codec = LabeledMarkerCodec::default();
        let labeled_marker_positions: Vec<LabeledMarker> = (0..labeled_marker_count)
            .map(|_| labeled_marker_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        log::debug!("Labeled Marker Positions: {:?}", labeled_marker_positions);
        let force_plate_count = src.get_u32_le();
        log::debug!("Force Plate Count: {}", force_plate_count);
        let force_plate_bytes = src.get_u32_le();
        log::debug!("Force Plate Bytes: {}", force_plate_bytes);
        let mut force_plate_codec = ForcePlateCodec::default();
        let force_plates: Vec<ForcePlate> = (0..force_plate_count)
            .map(|_| force_plate_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        log::debug!("Force Plates: {:?}", force_plates);
        let device_count = src.get_u32_le();
        log::debug!("Device Count: {}", device_count);
        let device_bytes = src.get_u32_le();
        log::debug!("Device Bytes: {}", device_bytes);
        let mut device_codec = DeviceCodec::default();
        let devices: Vec<Device> = (0..device_count)
            .map(|_| device_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        log::debug!("Devices: {:?}", devices);
        let timecode = src.get_u32_le();
        log::debug!("TimeCode: {}", timecode);
        let timecode_sub = src.get_u32_le();
        log::debug!("TimeCode Sub: {}", timecode_sub);
        let mut stamps_codec = StampsCodec::default();
        let stamps: Stamps = stamps_codec.decode(src).unwrap_or_default();
        log::debug!("Stamps: {:?}", stamps);
        let mut frame_parameters_codec = FrameParametersCodec::default();
        let frame_parameters: FrameParameters = frame_parameters_codec
            .decode(src)
            .unwrap_or(FrameParameters::Unrecognized);

        Ok(FrameData {
            packet_size,
            frame_number,
            markerset_count,
            markerset_bytes,
            markersets,
            unlabeled_marker_count,
            unlabeled_marker_bytes,
            unlabeled_marker_positions,
            rigid_body_count,
            rigid_body_bytes,
            rigid_bodies,
            skeleton_count,
            skeleton_bytes,
            skeletons,
            labeled_marker_count,
            labeled_marker_bytes,
            labeled_marker_positions,
            asset_count,
            asset_bytes,
            assets,
            force_plate_count,
            force_plate_bytes,
            force_plates,
            device_count,
            device_bytes,
            devices,
            timecode,
            timecode_sub,
            stamps,
            frame_parameters,
        })
    }
}

#[derive(Debug, Clone)]
pub struct FrameData {
    pub packet_size: u16,
    pub frame_number: u32,
    pub markerset_count: u32,
    pub markerset_bytes: u32,
    pub markersets: Vec<MarkerSet>,
    pub unlabeled_marker_count: u32,
    pub unlabeled_marker_bytes: u32,
    pub unlabeled_marker_positions: Vec<Vec3>,
    pub rigid_body_count: u32,
    pub rigid_body_bytes: u32,
    pub rigid_bodies: Vec<RigidBody>,
    pub skeleton_count: u32,
    pub skeleton_bytes: u32,
    pub skeletons: Vec<Skeleton>,
    pub labeled_marker_count: u32,
    pub labeled_marker_bytes: u32,
    pub labeled_marker_positions: Vec<LabeledMarker>,
    pub asset_count: u32,
    pub asset_bytes: u32,
    pub assets: Vec<Asset>,
    pub force_plate_count: u32,
    pub force_plate_bytes: u32,
    pub force_plates: Vec<ForcePlate>,
    pub device_count: u32,
    pub device_bytes: u32,
    pub devices: Vec<Device>,
    pub timecode: u32,
    pub timecode_sub: u32,
    pub stamps: Stamps,
    pub frame_parameters: FrameParameters,
}

#[derive(Debug, Default)]
pub struct ModelDefCodec;

impl Decoder for ModelDefCodec {
    type Item = ModelDef;
    type Error = Box<dyn error::Error>;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        let packet_size = src.get_u16_le();
        log::debug!("Packet Size: {} bytes", packet_size);
        let dataset_count = src.get_u32_le();
        let mut dataset = Vec::new();
        log::debug!("DataSet Count: {}", dataset_count);
        for _ in 0..dataset_count {
            let data_type = src.get_u32_le();
            log::debug!("Data Type: {}", data_type);
            let size = src.get_u32_le();
            log::debug!("Data Size: {}", size);
            let data = match data_type {
                0 => {
                    let mut codec = MarkerSetDescCodec;
                    ModelDefData::MarkerSetDesc {
                        size,
                        data: Box::new(codec.decode(src)?),
                    }
                }
                1 => {
                    let mut codec = RigidBodyDescCodec;
                    ModelDefData::RigidBodyDesc {
                        size,
                        data: Box::new(codec.decode(src)?),
                    }
                }
                5 => {
                    let mut codec = CameraDescCodec;
                    ModelDefData::CameraDesc {
                        size,
                        data: Box::new(codec.decode(src)?),
                    }
                }
                _ => unimplemented!(),
            };
            dataset.push(data);
        }

        Ok(ModelDef {
            packet_size,
            dataset_count,
            dataset,
        })
    }
}

#[derive(Debug, Clone)]
pub struct ModelDef {
    pub packet_size: u16,
    pub dataset_count: u32,
    pub dataset: Vec<ModelDefData>,
}

#[derive(Debug, Clone)]
pub enum ModelDefData {
    MarkerSetDesc { size: u32, data: Box<MarkerSetDesc> },
    RigidBodyDesc { size: u32, data: Box<RigidBodyDesc> },
    SkeletonDesc,
    ForcePlateDesc,
    DeviceDesc,
    CameraDesc { size: u32, data: Box<CameraDesc> },
    AssetDesc,
    Unknown,
}

#[derive(Debug, Default)]
pub struct Vec3Codec;

impl Encoder<Vec3> for Vec3Codec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: Vec3, dst: &mut BytesMut) -> Result<(), Self::Error> {
        dst.extend_from_slice(&bincode::serialize(&item)?);
        Ok(())
    }
}

impl Decoder for Vec3Codec {
    type Item = Vec3;
    type Error = Box<dyn error::Error>;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        Ok(Vec3 {
            x: src.get_f32_le(),
            y: src.get_f32_le(),
            z: src.get_f32_le(),
        })
    }
}

#[derive(Debug, Default)]
pub struct QuatCodec;

impl Encoder<Quat> for QuatCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: Quat, dst: &mut BytesMut) -> Result<(), Self::Error> {
        dst.extend_from_slice(&bincode::serialize(&item)?);
        Ok(())
    }
}

impl Decoder for QuatCodec {
    type Item = Quat;
    type Error = Box<dyn error::Error>;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        Ok(Quat::from_xyzw(
            src.get_f32_le(),
            src.get_f32_le(),
            src.get_f32_le(),
            src.get_f32_le(),
        )
        .normalize())
    }
}

/* Marker Asset */

#[derive(Debug, Default)]
pub struct MarkerAssetCodec {}

impl Encoder<MarkerAsset> for MarkerAssetCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: MarkerAsset, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // Reserve enough space for at least the id, rigid body count, and marker count
        dst.reserve(3 * 8);
        dst.extend_from_slice(&item.id.to_be_bytes());
        if item.rigid_body_count != item.rigid_bodies.len() as u32 {
            log::warn!(
                "RigidBody count {} does not match length of rigid_bodies vec {}",
                item.rigid_body_count,
                item.rigid_bodies.len()
            );
            dst.extend_from_slice(&item.rigid_body_count.to_le_bytes()[..]);
        } else {
            dst.extend_from_slice(&(item.rigid_bodies.len() as u32).to_le_bytes()[..]);
        }
        let mut rigid_body_codec = RigidBodyCodec::default();
        for rb in item.rigid_bodies.into_iter() {
            rigid_body_codec.encode(rb, dst)?;
        }
        item.marker_positions.iter().for_each(|p| {
            dst.extend_from_slice(&p.x.to_le_bytes()[..]);
            dst.extend_from_slice(&p.y.to_le_bytes()[..]);
            dst.extend_from_slice(&p.z.to_le_bytes()[..]);
        });
        Ok(())
    }
}

impl Decoder for MarkerAssetCodec {
    type Error = Box<dyn error::Error>;
    type Item = MarkerAsset;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        let id = src.get_u32_le();

        let rigid_body_count = src.get_u32_le();
        let mut rigidbody_codec = RigidBodyCodec::default();
        let rigid_bodies: Vec<RigidBody> = (0..rigid_body_count)
            .map(|_| rigidbody_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        let marker_count = src.get_u32_le();
        let marker_positions = (0..marker_count)
            .map(|_| Vec3 {
                x: src.get_f32_le(),
                y: src.get_f32_le(),
                z: src.get_f32_le(),
            })
            .collect();

        Ok(Self::Item {
            id,
            rigid_body_count,
            rigid_bodies,
            marker_count,
            marker_positions,
        })
    }
}

#[derive(Debug, Clone, Default)]
pub struct MarkerAsset {
    pub id: u32,
    pub rigid_body_count: u32,
    pub rigid_bodies: Vec<RigidBody>,
    pub marker_count: u32,
    pub marker_positions: Vec<Vec3>,
}

/* MarkerSet */

#[derive(Debug, Default)]
pub struct MarkerSetCodec {}

impl Encoder<MarkerSet> for MarkerSetCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: MarkerSet, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least the name, marker count, and a single position
        dst.reserve(item.name.len() + 16);
        dst.extend_from_slice(item.name.as_bytes());
        // end string with null terminator
        dst.put_u8(0);
        if item.marker_count != item.positions.len() as u32 {
            log::warn!(
                "Marker count {} does not match length of marker vec {}",
                item.marker_count,
                item.positions.len()
            );
            dst.extend_from_slice(&item.marker_count.to_le_bytes()[..]);
        } else {
            dst.extend_from_slice(&(item.positions.len() as u32).to_le_bytes()[..]);
        }
        item.positions.iter().for_each(|p| {
            dst.extend_from_slice(&p.x.to_le_bytes()[..]);
            dst.extend_from_slice(&p.y.to_le_bytes()[..]);
            dst.extend_from_slice(&p.z.to_le_bytes()[..]);
        });
        Ok(())
    }
}

impl Decoder for MarkerSetCodec {
    type Error = Box<dyn error::Error>;
    type Item = MarkerSet;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        let mut name_buf = Vec::new();
        let _len = src.reader().read_until(b'\0', &mut name_buf)?;
        let name = String::from_utf8(name_buf)?;

        if src.remaining() < 16 {
            return Err("Not enough bytest to decode MarkerSet".into());
        }
        log::debug!("MarkerSet name: '{}'", name);

        let marker_count = src.get_u32_le();
        log::debug!("Marker count: {}", marker_count);
        let positions = (0..marker_count)
            .map(|_| Vec3 {
                x: src.get_f32_le(),
                y: src.get_f32_le(),
                z: src.get_f32_le(),
            })
            .collect();

        Ok(Self::Item {
            name,
            marker_count,
            positions,
        })
    }
}

#[derive(Debug, Clone)]
pub struct MarkerSet {
    pub name: String,
    pub marker_count: u32,
    pub positions: Vec<Vec3>,
}

impl MarkerSet {
    pub fn new(name: &str, marker_count: u32) -> Self {
        Self {
            name: name.to_string(),
            marker_count,
            positions: Vec::new(),
        }
    }
}

/* RigidBody */

#[derive(Debug, Default)]
pub struct RigidBodyCodec {}

impl Encoder<RigidBody> for RigidBodyCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: RigidBody, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least the id, pos, and rot
        dst.reserve(38);
        dst.extend_from_slice(&item.id.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.x.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.y.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.z.to_le_bytes()[..]);
        dst.extend_from_slice(&item.rot.x.to_le_bytes()[..]);
        dst.extend_from_slice(&item.rot.y.to_le_bytes()[..]);
        dst.extend_from_slice(&item.rot.z.to_le_bytes()[..]);
        dst.extend_from_slice(&item.rot.w.to_le_bytes()[..]);
        dst.extend_from_slice(&item.mean_marker_err.to_le_bytes()[..]);
        Ok(())
    }
}

impl Decoder for RigidBodyCodec {
    type Error = Box<dyn error::Error>;
    type Item = RigidBody;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        if src.remaining() < 32 {
            return Err("Not enough bytes to decode RigidBody".into());
        }

        let id = src.get_u32_le();
        let pos = Vec3 {
            x: src.get_f32_le(),
            y: src.get_f32_le(),
            z: src.get_f32_le(),
        };
        let rot = Quat::from_xyzw(
            src.get_f32_le(),
            src.get_f32_le(),
            src.get_f32_le(),
            src.get_f32_le(),
        )
        .normalize();

        let mean_marker_err = src.get_f32_le();
        let is_tracking_valid = (src.get_u16_le() & 0x01) != 0;

        Ok(RigidBody {
            id,
            pos,
            rot,
            is_tracking_valid,
            mean_marker_err,
        })
    }
}

#[derive(Debug, Clone)]
pub struct RigidBody {
    pub id: u32,
    pub pos: Vec3,
    pub rot: Quat,
    pub is_tracking_valid: bool,
    pub mean_marker_err: f32,
}

impl RigidBody {
    pub fn rub_to_frd(self) -> Self {
        Self {
            pos: glam::vec3(self.pos.x, self.pos.z, -self.pos.y),
            ..self
        }
    }
}

/* RigidBodyAsset */

#[derive(Debug, Default)]
pub struct RigidBodyAssetCodec {}

impl Encoder<RigidBodyAsset> for RigidBodyAssetCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: RigidBodyAsset, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // Reserve enough space for at least the id, pos, rot, marker error, and param
        dst.reserve(38);
        dst.extend_from_slice(&item.id.to_le_bytes()[..]);
        dst.extend_from_slice(&bincode::serialize(&item.pos)?);
        dst.extend_from_slice(&bincode::serialize(&item.rot)?);
        dst.extend_from_slice(&item.marker_error.to_le_bytes()[..]);
        dst.extend_from_slice(&(item.param).to_le_bytes()[..]);

        Ok(())
    }
}

impl Decoder for RigidBodyAssetCodec {
    type Error = Box<dyn error::Error>;
    type Item = RigidBodyAsset;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        if src.remaining() < 38 {
            return Err("Not enough bytes to decode RigidBodyAsset".into());
        }

        let id = src.get_u32_le();
        let pos = Vec3 {
            x: src.get_f32_le(),
            y: src.get_f32_le(),
            z: src.get_f32_le(),
        };
        let rot = Quat::from_xyzw(
            src.get_f32_le(),
            src.get_f32_le(),
            src.get_f32_le(),
            src.get_f32_le(),
        )
        .normalize();

        let marker_error = src.get_f32_le();
        let param = src.get_i16_le();
        Ok(RigidBodyAsset {
            id,
            pos,
            rot,
            marker_error,
            param,
        })
    }
}

#[derive(Debug, Clone)]
pub struct RigidBodyAsset {
    pub id: u32,
    pub pos: Vec3,
    pub rot: Quat,
    pub marker_error: f32,
    pub param: i16,
}

/* Skeleton */

#[derive(Debug, Default)]
pub struct SkeletonCodec {}

impl Encoder<Skeleton> for SkeletonCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: Skeleton, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least the id and rigidbody count
        dst.reserve(8);
        if item.rigid_body_count != item.rigid_bodies.len() as u32 {
            log::warn!(
                "RigidBody count {} does not match length of rigid_bodies vec {}",
                item.rigid_body_count,
                item.rigid_bodies.len()
            );
            dst.extend_from_slice(&item.rigid_body_count.to_le_bytes()[..]);
        } else {
            dst.extend_from_slice(&(item.rigid_bodies.len() as u32).to_le_bytes()[..]);
        }
        let mut rigid_body_codec = RigidBodyCodec::default();
        for rb in item.rigid_bodies.into_iter() {
            rigid_body_codec.encode(rb, dst)?;
        }
        Ok(())
    }
}

impl Decoder for SkeletonCodec {
    type Error = Box<dyn error::Error>;
    type Item = Skeleton;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        if src.remaining() < 8 {
            return Err("Not enough bytes to decode Skeleton".into());
        }
        let id = src.get_u32_le();
        log::debug!("Skeleton ID: {}", id);
        let rigid_body_count = src.get_u32_le();
        log::debug!("Skeleton RigidBody Count: {}", rigid_body_count);
        let mut rigidbody_codec = RigidBodyCodec::default();
        let rigid_bodies: Vec<RigidBody> = (0..rigid_body_count)
            .map(|_| rigidbody_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        log::debug!("Skeleton RigidBodies: {:?}", rigid_bodies);
        Ok(Skeleton {
            id,
            rigid_body_count,
            rigid_bodies,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Skeleton {
    pub id: u32,
    pub rigid_body_count: u32,
    pub rigid_bodies: Vec<RigidBody>,
}

#[derive(Debug, Default)]
pub struct AssetCodec {}

impl Encoder<Asset> for AssetCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: Asset, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least the id and rigidbody count
        dst.reserve(8);
        if item.rigid_body_count != item.rigid_bodies.len() as u32 {
            log::warn!(
                "RigidBody count {} does not match length of rigid_bodies vec {}",
                item.rigid_body_count,
                item.rigid_bodies.len()
            );
            dst.extend_from_slice(&item.rigid_body_count.to_le_bytes()[..]);
        } else {
            dst.extend_from_slice(&(item.rigid_bodies.len() as u32).to_le_bytes()[..]);
        }
        let mut rigid_body_codec = RigidBodyAssetCodec::default();
        for rb in item.rigid_bodies.into_iter() {
            rigid_body_codec.encode(rb, dst)?;
        }
        Ok(())
    }
}

impl Decoder for AssetCodec {
    type Error = Box<dyn error::Error>;
    type Item = Asset;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        if src.remaining() < 8 {
            return Err("Not enough bytes to decode Asset".into());
        }
        let id = src.get_u32_le();
        let rigid_body_count = src.get_u32_le();
        let mut rigidbody_codec = RigidBodyAssetCodec::default();
        let rigid_bodies: Vec<RigidBodyAsset> = (0..rigid_body_count)
            .map(|_| rigidbody_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Asset {
            id,
            rigid_body_count,
            rigid_bodies,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Asset {
    pub id: u32,
    pub rigid_body_count: u32,
    pub rigid_bodies: Vec<RigidBodyAsset>,
}

/* LabeledMarker */

#[derive(Debug, Default)]
pub struct LabeledMarkerCodec {}

impl Encoder<LabeledMarker> for LabeledMarkerCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: LabeledMarker, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for entire struct
        dst.reserve(26);
        dst.extend_from_slice(&item.id.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.x.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.y.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.z.to_le_bytes()[..]);
        dst.extend_from_slice(&item.size.to_le_bytes()[..]);
        match item.status {
            LabeledMarkerStatus::Occluded => dst.extend_from_slice(&1_u16.to_le_bytes()[..]),
            LabeledMarkerStatus::PointCloudSolved => {
                dst.extend_from_slice(&2_u16.to_le_bytes()[..])
            }
            LabeledMarkerStatus::ModelSolved => dst.extend_from_slice(&4_u16.to_le_bytes()[..]),
            LabeledMarkerStatus::Unrecognized => dst.extend_from_slice(&0_u16.to_le_bytes()[..]),
        };
        Ok(())
    }
}

impl Decoder for LabeledMarkerCodec {
    type Error = Box<dyn error::Error>;
    type Item = LabeledMarker;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        if src.remaining() < 26 {
            return Err("Not enough bytes to decode LabeledMarker".into());
        }
        let id = src.get_u32_le();
        let pos = Vec3 {
            x: src.get_f32_le(),
            y: src.get_f32_le(),
            z: src.get_f32_le(),
        };
        let size = src.get_f32_le();
        let status = match src.get_u16_le() {
            0x01 => LabeledMarkerStatus::Occluded,
            0x02 => LabeledMarkerStatus::PointCloudSolved,
            0x04 => LabeledMarkerStatus::ModelSolved,
            _ => LabeledMarkerStatus::Unrecognized,
        };
        let residual = src.get_f32_le();
        Ok(LabeledMarker {
            id,
            pos,
            size,
            status,
            residual,
        })
    }
}

#[derive(Debug, Clone)]
pub struct LabeledMarker {
    pub id: u32,
    pub pos: Vec3,
    pub size: f32,
    pub status: LabeledMarkerStatus,
    pub residual: f32,
}

#[derive(Debug, Copy, Clone)]
pub enum LabeledMarkerStatus {
    Occluded,
    PointCloudSolved,
    ModelSolved,
    Unrecognized,
}

#[derive(Debug, Default)]
pub struct ForcePlateCodec {}

impl Encoder<ForcePlate> for ForcePlateCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: ForcePlate, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least id and channel count
        dst.reserve(8);
        dst.extend_from_slice(&item.id.to_le_bytes()[..]);
        dst.extend_from_slice(&item.channel_count.to_le_bytes()[..]);
        let mut force_plate_channel_codec = ForcePlateChannelCodec::default();
        for ch in item.channels.into_iter() {
            force_plate_channel_codec.encode(ch, dst)?;
        }
        Ok(())
    }
}

impl Decoder for ForcePlateCodec {
    type Error = Box<dyn error::Error>;
    type Item = ForcePlate;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        if src.remaining() < 8 {
            return Err("Not enough bytes to decode ForcePlate".into());
        }

        let id = src.get_u32_le();
        let channel_count = src.get_u32_le();
        let mut force_plate_channel_codec = ForcePlateChannelCodec::default();
        let channels = (0..channel_count)
            .map(|_| force_plate_channel_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(ForcePlate {
            id,
            channel_count,
            channels,
        })
    }
}

#[derive(Debug, Clone)]
pub struct ForcePlate {
    pub id: u32,
    pub channel_count: u32,
    pub channels: Vec<ForcePlateChannel>,
}

#[derive(Debug, Default)]
pub struct ForcePlateChannelCodec {}

impl Encoder<ForcePlateChannel> for ForcePlateChannelCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: ForcePlateChannel, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least value count and 1 value
        dst.reserve(8);
        dst.extend_from_slice(&item.value_count.to_le_bytes()[..]);
        item.values
            .into_iter()
            .for_each(|v| dst.extend_from_slice(&v.to_le_bytes()[..]));
        Ok(())
    }
}

impl Decoder for ForcePlateChannelCodec {
    type Error = Box<dyn error::Error>;
    type Item = ForcePlateChannel;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        if src.remaining() < 4 {
            return Err("Not enough bytes to decode ForcePlateChannel".into());
        }
        let value_count = src.get_u32_le();
        let values = (0..value_count).map(|_| src.get_u32_le()).collect();
        Ok(ForcePlateChannel {
            value_count,
            values,
        })
    }
}
#[derive(Debug, Clone)]
pub struct ForcePlateChannel {
    pub value_count: u32,
    pub values: Vec<u32>,
}

#[derive(Debug, Default)]
pub struct DeviceCodec {}

impl Encoder<Device> for DeviceCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: Device, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least id and channel count
        dst.reserve(8);
        dst.extend_from_slice(&item.id.to_le_bytes()[..]);
        dst.extend_from_slice(&item.channel_count.to_le_bytes()[..]);
        let mut device_channel_codec = DeviceChannelCodec::default();
        for ch in item.channels.into_iter() {
            device_channel_codec.encode(ch, dst)?;
        }
        Ok(())
    }
}

impl Decoder for DeviceCodec {
    type Error = Box<dyn error::Error>;
    type Item = Device;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        // must have at least an id and a channel count
        if src.remaining() < 8 {
            return Err("Not enough bytes to decode Device".into());
        }
        let id = src.get_u32_le();
        let channel_count = src.get_u32_le();
        let mut device_channel_codec = DeviceChannelCodec::default();
        let channels = (0..channel_count)
            .map(|_| device_channel_codec.decode(src))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Device {
            id,
            channel_count,
            channels,
        })
    }
}
#[derive(Debug, Clone)]
pub struct Device {
    pub id: u32,
    pub channel_count: u32,
    pub channels: Vec<DeviceChannel>,
}

#[derive(Debug, Default)]
pub struct DeviceChannelCodec {}

impl Encoder<DeviceChannel> for DeviceChannelCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: DeviceChannel, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least value count and a single value
        dst.reserve(8);
        dst.extend_from_slice(&item.value_count.to_le_bytes()[..]);
        item.values
            .into_iter()
            .for_each(|v| dst.extend_from_slice(&v.to_le_bytes()[..]));
        Ok(())
    }
}

impl Decoder for DeviceChannelCodec {
    type Error = Box<dyn error::Error>;
    type Item = DeviceChannel;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        // must have at least a count and a single value
        if src.remaining() < 8 {
            return Err("Not enough bytes to decode DeviceChannel".into());
        }
        let value_count = src.get_u32_le();
        let values = (0..value_count).map(|_| src.get_u32_le()).collect();
        Ok(DeviceChannel {
            value_count,
            values,
        })
    }
}

#[derive(Debug, Clone)]
pub struct DeviceChannel {
    pub value_count: u32,
    pub values: Vec<u32>,
}

#[derive(Debug, Default)]
pub struct StampsCodec {}

impl Encoder<Stamps> for StampsCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: Stamps, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for entire struct
        dst.reserve(32);
        dst.extend_from_slice(&item.timestamp.to_le_bytes()[..]);
        dst.extend_from_slice(&item.timestamp_mid.to_le_bytes()[..]);
        dst.extend_from_slice(&item.timestamp_recv.to_le_bytes()[..]);
        dst.extend_from_slice(&item.timestamp_tx.to_le_bytes()[..]);
        dst.extend_from_slice(&item.timestamp_precision.to_le_bytes()[..]);
        dst.extend_from_slice(&item.timestamp_precision_fraction.to_le_bytes()[..]);
        let mut frame_param_codec = FrameParametersCodec::default();
        frame_param_codec.encode(item.param, dst)?;
        Ok(())
    }
}

impl Decoder for StampsCodec {
    type Error = Box<dyn error::Error>;
    type Item = Stamps;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        if src.remaining() < 42 {
            return Err("Not enough bytes to decode Stamps".into());
        }
        let timestamp = src.get_f64_le();
        log::debug!("Timestamp: {}", timestamp);
        let timestamp_mid = src.get_f64_le();
        log::debug!("Timestamp Mid: {}", timestamp_mid);
        let timestamp_recv = src.get_f64_le();
        log::debug!("Timestamp Recv: {}", timestamp_recv);
        let timestamp_tx = src.get_f64_le();
        log::debug!("Timestamp Tx: {}", timestamp_tx);
        let timestamp_precision = src.get_i32_le();
        log::debug!("Timestamp Precision: {}", timestamp_precision);
        let timestamp_precision_fraction = src.get_i32_le();
        log::debug!(
            "Timestamp Precision Fraction: {}",
            timestamp_precision_fraction
        );

        let mut frame_param_codec = FrameParametersCodec::default();
        let param: FrameParameters = match frame_param_codec.decode(src) {
            Ok(fp) => fp,
            _ => FrameParameters::Unrecognized,
        };
        log::debug!("Timestamp Parameter: {:?}", param);

        Ok(Stamps {
            timestamp,
            timestamp_mid,
            timestamp_recv,
            timestamp_tx,
            timestamp_precision,
            timestamp_precision_fraction,
            param,
        })
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Stamps {
    pub timestamp: f64,
    pub timestamp_mid: f64,
    pub timestamp_recv: f64,
    pub timestamp_tx: f64,
    pub timestamp_precision: i32,
    pub timestamp_precision_fraction: i32,
    pub param: FrameParameters,
}

impl Default for Stamps {
    fn default() -> Self {
        Self {
            timestamp: 0.0,
            timestamp_mid: 0.0,
            timestamp_recv: 0.0,
            timestamp_tx: 0.0,
            timestamp_precision: 0,
            timestamp_precision_fraction: 0,
            param: FrameParameters::Unrecognized,
        }
    }
}

#[derive(Debug, Default)]
pub struct FrameParametersCodec {}

impl Encoder<FrameParameters> for FrameParametersCodec {
    type Error = Box<dyn error::Error>;
    fn encode(&mut self, item: FrameParameters, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least value count and 1 value
        dst.reserve(2);
        match item {
            FrameParameters::IsRecording => dst.extend_from_slice(&1_u16.to_le_bytes()[..]),
            FrameParameters::TrackingModelsChanged => {
                dst.extend_from_slice(&2_u16.to_le_bytes()[..])
            }
            _ => dst.extend_from_slice(&0_u16.to_le_bytes()[..]),
        }
        Ok(())
    }
}

impl Decoder for FrameParametersCodec {
    type Error = Box<dyn error::Error>;
    type Item = FrameParameters;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        if src.remaining() < 2 {
            return Err("Not enough bytes to decode FrameParameters".into());
        }
        match src.get_u16() {
            0x01 => Ok(FrameParameters::IsRecording),
            0x02 => Ok(FrameParameters::TrackingModelsChanged),
            _ => Ok(FrameParameters::Unrecognized),
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(u16)]
pub enum FrameParameters {
    IsRecording,
    TrackingModelsChanged,
    Unrecognized,
}

/* MarkerSetDesc */

#[derive(Debug, Default)]
pub struct MarkerSetDescCodec;

impl Encoder<MarkerSetDesc> for MarkerSetDescCodec {
    type Error = Box<dyn std::error::Error>;
    fn encode(&mut self, item: MarkerSetDesc, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least the name, marker count, and a single position
        dst.reserve(item.name.len() + 16);
        dst.extend_from_slice(item.name.as_bytes());
        // end string with null terminator
        dst.put_u8(0);
        if item.marker_count != item.marker_names.len() as i32 {
            log::warn!(
                "Marker count {} does not match length of marker vec {}",
                item.marker_count,
                item.marker_names.len()
            );
            dst.extend_from_slice(&item.marker_count.to_le_bytes()[..]);
        } else {
            dst.extend_from_slice(&(item.marker_names.len() as i32).to_le_bytes()[..]);
        }
        item.marker_names.iter().for_each(|n| {
            dst.extend_from_slice(n.as_bytes());
        });
        Ok(())
    }
}

impl Decoder for MarkerSetDescCodec {
    type Error = Box<dyn std::error::Error>;
    type Item = MarkerSetDesc;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        let mut name_buf = Vec::new();
        let _len = src.reader().read_until(b'\0', &mut name_buf)?;
        let name = String::from_utf8(name_buf)?;

        if src.remaining() < 16 {
            let msg = "Not enough bytest to decode MarkerSetDesc";
            log::error!("{}", msg);
            return Err(msg.into());
        }
        log::debug!("MarkerSet name: '{}'", name);

        let marker_count = src.get_i32_le();
        log::debug!("Marker count: {}", marker_count);

        let mut marker_names = Vec::new();
        for _ in 0..marker_count {
            let mut name_buf = Vec::new();
            let _len = src.reader().read_until(b'\0', &mut name_buf)?;
            marker_names.push(String::from_utf8(name_buf)?);
        }

        Ok(Self::Item {
            name,
            marker_count,
            marker_names,
        })
    }
}

#[derive(Debug, Clone)]
pub struct MarkerSetDesc {
    pub name: String,
    pub marker_count: i32,
    pub marker_names: Vec<String>,
}

impl MarkerSetDesc {
    pub fn new(name: &str, marker_count: i32) -> Self {
        Self {
            name: name.to_string(),
            marker_count,
            marker_names: Vec::new(),
        }
    }
}

/* RigidBodyDesc */

#[derive(Debug, Default)]
pub struct RigidBodyDescCodec;

impl Encoder<RigidBodyDesc> for RigidBodyDescCodec {
    type Error = Box<dyn std::error::Error>;
    fn encode(&mut self, item: RigidBodyDesc, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least the id, pos, and rot
        //dst.reserve(38);
        dst.extend_from_slice(item.name.as_bytes());
        dst.extend_from_slice(&item.id.to_le_bytes()[..]);
        dst.extend_from_slice(&item.parent_id.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.x.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.y.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.z.to_le_bytes()[..]);
        dst.extend_from_slice(&item.marker_count.to_le_bytes()[..]);
        item.marker_offsets.iter().for_each(|m| {
            dst.extend_from_slice(&m.x.to_le_bytes()[..]);
            dst.extend_from_slice(&m.x.to_le_bytes()[..]);
            dst.extend_from_slice(&m.x.to_le_bytes()[..]);
        });
        item.marker_active_labels.iter().for_each(|m| {
            dst.extend_from_slice(&m.to_le_bytes()[..]);
            dst.extend_from_slice(&m.to_le_bytes()[..]);
            dst.extend_from_slice(&m.to_le_bytes()[..]);
        });
        item.marker_names.iter().for_each(|m| {
            dst.extend_from_slice(m.as_bytes());
            dst.extend_from_slice(m.as_bytes());
            dst.extend_from_slice(m.as_bytes());
        });
        Ok(())
    }
}

impl Decoder for RigidBodyDescCodec {
    type Error = Box<dyn std::error::Error>;
    type Item = RigidBodyDesc;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        let mut name_buf = Vec::new();
        let _len = src.reader().read_until(b'\0', &mut name_buf)?;
        let name = String::from_utf8(name_buf)?;
        log::debug!("RigidBodyDesc name: '{}'", name);

        let id = src.get_i32_le();
        let parent_id = src.get_i32_le();

        let pos = Vec3 {
            x: src.get_f32_le(),
            y: src.get_f32_le(),
            z: src.get_f32_le(),
        };

        let marker_count = src.get_i32_le();

        let marker_offsets = (0..marker_count)
            .map(|_| Vec3 {
                x: src.get_f32_le(),
                y: src.get_f32_le(),
                z: src.get_f32_le(),
            })
            .collect();

        let marker_active_labels = (0..marker_count).map(|_| src.get_i32_le()).collect();

        let mut marker_names = Vec::new();
        for _ in 0..marker_count {
            let mut name_buf = Vec::new();
            let _len = src.reader().read_until(b'\0', &mut name_buf)?;
            marker_names.push(String::from_utf8(name_buf)?);
        }

        Ok(RigidBodyDesc {
            name,
            id,
            parent_id,
            pos,
            marker_count,
            marker_offsets,
            marker_active_labels,
            marker_names,
        })
    }
}

#[derive(Debug, Clone)]
pub struct RigidBodyDesc {
    pub name: String,
    pub id: i32,
    pub parent_id: i32,
    pub pos: Vec3,
    pub marker_count: i32,
    pub marker_offsets: Vec<Vec3>,
    pub marker_active_labels: Vec<i32>,
    pub marker_names: Vec<String>,
}

/* CameraDesc */

#[derive(Debug, Default)]
pub struct CameraDescCodec;

impl Encoder<CameraDesc> for CameraDescCodec {
    type Error = Box<dyn std::error::Error>;
    fn encode(&mut self, item: CameraDesc, dst: &mut BytesMut) -> Result<(), Self::Error> {
        // reserve enough space for at least the id, pos, and rot
        dst.reserve(item.name.len() + 28);
        dst.extend_from_slice(item.name.as_bytes());
        dst.extend_from_slice(&item.pos.x.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.y.to_le_bytes()[..]);
        dst.extend_from_slice(&item.pos.z.to_le_bytes()[..]);
        dst.extend_from_slice(&item.rot.x.to_le_bytes()[..]);
        dst.extend_from_slice(&item.rot.y.to_le_bytes()[..]);
        dst.extend_from_slice(&item.rot.z.to_le_bytes()[..]);
        dst.extend_from_slice(&item.rot.w.to_le_bytes()[..]);
        Ok(())
    }
}

impl Decoder for CameraDescCodec {
    type Error = Box<dyn std::error::Error>;
    type Item = CameraDesc;
    fn decode(&mut self, src: &mut BytesMut) -> Result<Self::Item, Self::Error> {
        let mut name_buf = Vec::new();
        let _len = src.reader().read_until(b'\0', &mut name_buf)?;
        let name = String::from_utf8(name_buf)?;
        log::debug!("CameraDesc name: {}", name);

        let pos = Vec3 {
            x: src.get_f32_le(),
            y: src.get_f32_le(),
            z: src.get_f32_le(),
        };
        log::debug!("CameraDesc pos: {}", pos);

        let rot = Quat::from_xyzw(
            src.get_f32_le(),
            src.get_f32_le(),
            src.get_f32_le(),
            src.get_f32_le(),
        );
        log::debug!("CameraDesc rot: {}", rot);

        Ok(CameraDesc { name, pos, rot })
    }
}

#[derive(Debug, Clone)]
pub struct CameraDesc {
    pub name: String,
    pub pos: Vec3,
    pub rot: Quat,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn init() {
        let _ = env_logger::builder()
            .filter_level(log::LevelFilter::Trace)
            .is_test(true)
            .try_init();
    }

    #[test]
    fn parse_frame() {
        init();
        let path = std::path::PathBuf::from("src/FrameData.bin");
        let packet = std::fs::read(path).unwrap();
        let buf = BytesMut::from(packet.as_slice());
        let message = Message::from_bytes(buf).expect("Failed to decode message from bytes");
        match message {
            Message::FrameData(frame) => {
                assert_eq!(frame.packet_size, 369);
                assert_eq!(frame.frame_number, 197792);
                assert_eq!(frame.markerset_count, 2);
                assert_eq!(frame.markerset_bytes, 209);
                assert_eq!(frame.unlabeled_marker_count, 0);
                assert_eq!(frame.unlabeled_marker_bytes, 0);
                assert_eq!(frame.rigid_body_count, 1);
                assert_eq!(frame.rigid_body_bytes, 38);
            }
            val => panic!("Expected FrameData, got {:?}", val),
        };
    }

    #[test]
    fn parse_modeldef() {
        init();
        let path = std::path::PathBuf::from("src/ModelDef.bin");
        let packet = std::fs::read(path).unwrap();
        let buf = BytesMut::from(packet.as_slice());
        let message = Message::from_bytes(buf);
        assert!(message.is_ok());
    }
}
