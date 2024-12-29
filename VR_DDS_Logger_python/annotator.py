import omni.replicator.core as rep
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from pxr import UsdGeom
from omni.isaac.core.prims.xform_prim import XFormPrim
class AnnotatorManager:
    def __init__(self, world):
        self.annotators = {}
        self.render_products = {}
        self.cameras = {}
        self.annotator_types = [
            "rgb",
            "distance_to_camera",
            "pointcloud",
            "semantic_segmentation",
        ]
        self.camera_info = {}
        self.world = world

    def registerCamera(
        self, parent_prim, name, translation, orientation, resolution=(1024, 600)
    ):
        """
        Register a camera in the scene. The camera will be attached to the parent_prim.
        parent_prim: The prim path of the parent prim. The camera will be attached to this prim.
        name: The name of the camera.
        translation: The translation of the camera relative to the parent prim in the form of (x, y, z).
        orientation: The orientation of the camera relative to the parent prim in the form of (x, y, z, w).
        resolution: The resolution of the camera in the form of (width, height).
        """
        qx, qy, qz, qw = tuple(orientation)
        xformprim = XFormPrim( prim_path=parent_prim + "/" + name,
                                name=name,
                                translation=tuple(translation),
                                orientation=(qw, qx, qy, qz) # Omniverse core convention (w, x, y, z)
                                )
        self.cameras[name] = self.world.stage.DefinePrim(
            xformprim.prim_path + "/" + name, "Camera"
        )
        UsdGeom.Xformable(self.cameras[name]).AddTranslateOp().Set((0.,0.,0.))
        #Rotate around x for 180 degrees to make the convention consistent with opencv/ROS
        UsdGeom.Xformable(self.cameras[name]).AddRotateXYZOp().Set((180.,0.,0.)) 
        self.camera_info[name] = {
            "translation": translation,
            "orientation": orientation,
            "resolution": resolution,
        }
        self.render_products[name] = rep.create.render_product(
            str(self.cameras[name].GetPrimPath()), resolution
        )

    def registerAnnotator(self, camera_name, type):
        assert (
            type in self.annotator_types
        ), "Requested replicator type not available. Choose from: " + str(
            self.annotator_types
        )
        assert (
            camera_name in self.cameras.keys()
        ), "The requested camera is not registered. Please register the camera first."
        self.annotators[camera_name + ":" + type] = rep.AnnotatorRegistry.get_annotator(
            type
        )
        self.annotators[camera_name + ":" + type].attach(
            [self.render_products[camera_name]]
        )

    def getData(self, annotator_name):
        assert (
            annotator_name in self.annotators.keys()
        ), "The requested annotator is not registered. Please register the annotator first."
        return self.annotators[annotator_name].get_data()

    def getStreams(self):
        return list(self.annotators.keys())

    def setFocalLength(self, name, value):
        assert (
            name in self.cameras.keys()
        ), "The requested camera is not registered. Please register the camera first."
        self.cameras[name].GetAttribute("focalLength").Set(value)

    def setClippingRange(self, name, min=0.2, max=1000000.0):
        assert (
            name in self.cameras.keys()
        ), "The requested camera is not registered. Please register the camera first."
        self.cameras[name].GetAttribute("clippingRange").Set((min, max))

    # TODO: implement these functions
    def getCameraIntrinsics(self, name):
        return None

    def getCameraExtrinsics(self, name):
        return None