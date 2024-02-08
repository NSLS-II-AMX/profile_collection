
from ophyd.areadetector.filestore_mixins import FileStorePluginBase
from ophyd.areadetector.plugins import JPEGPlugin, PluginBase, register_plugin
from ophyd.areadetector.base import DDC_EpicsSignal, DDC_EpicsSignalRO
from ophyd.areadetector.base import ADComponent as ADCpt
print(f"Loading {__file__}")


class FileStoreJPEG(FileStorePluginBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.filestore_spec = "AD_JPEG"  # spec name stored in resource doc
        self.stage_sigs.update(
            [
                ("file_template", "%s%s_%6.6d.jpg"),
                ("file_write_mode", "Single"),
            ]
        )
        # 'Single' file_write_mode means one image : one file.
        # It does NOT mean that 'num_images' is ignored.

    def get_frames_per_point(self):
        return self.parent.cam.num_images.get()

    def stage(self):
        super().stage()
        # this over-rides the behavior is the base stage
        self._fn = self._fp

        resource_kwargs = {
            "template": self.file_template.get(),
            "filename": self.file_name.get(),
            "frame_per_point": self.get_frames_per_point(),
        }
        self._generate_resource(resource_kwargs)


class JPEGPluginWithFileStore(JPEGPlugin, FileStoreJPEG):
    pass


@register_plugin
class CVPlugin(PluginBase):
    _default_suffix = "CV1:"
    _suffix_re = "CV1\d:"
    _default_read_attrs = ["outputs"]
    func_sets = DDC_EpicsSignal(
        *[(f"func_set{k}", f"CompVisionFunction{k}") for k in range(1, 4)]
    )
    inputs = DDC_EpicsSignal(
        *[(f"input{k}", f"Input{k}") for k in range(1, 11)]
    )
    outputs = DDC_EpicsSignalRO(
        *[(f"output{k}", f"Output{k}_RBV") for k in range(1, 11)]
    )
    cam_depth = ADCpt(EpicsSignal, "CompVisionCamDepth", kind="config")
