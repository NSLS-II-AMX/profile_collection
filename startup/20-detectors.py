print(f"Loading {__file__}")

from ophyd import (SingleTrigger, TIFFPlugin, ProsilicaDetector,
                   ImagePlugin, StatsPlugin, ROIPlugin, DetectorBase, HDF5Plugin,
                   TransformPlugin, ProcessPlugin, AreaDetector, EpicsSignalRO)

import ophyd.areadetector.cam as cam

from ophyd.areadetector.filestore_mixins import (FileStoreTIFFIterativeWrite,
                                                 FileStoreHDF5IterativeWrite)

from ophyd import Component as Cpt

class TIFFPluginWithFileStore(TIFFPlugin, FileStoreTIFFIterativeWrite):
    pass


class StandardProsilica(SingleTrigger, ProsilicaDetector):
    image = Cpt(ImagePlugin, 'image1:')
    roi1 = Cpt(ROIPlugin, 'ROI1:')
    roi2 = Cpt(ROIPlugin, 'ROI2:')
    roi3 = Cpt(ROIPlugin, 'ROI3:')
    roi4 = Cpt(ROIPlugin, 'ROI4:')
    trans1 = Cpt(TransformPlugin, 'Trans1:')
    proc1 = Cpt(ProcessPlugin, 'Proc1:')
    stats1 = Cpt(StatsPlugin, 'Stats1:')
    stats2 = Cpt(StatsPlugin, 'Stats2:')
    stats3 = Cpt(StatsPlugin, 'Stats3:')
    stats4 = Cpt(StatsPlugin, 'Stats4:')
    stats5 = Cpt(StatsPlugin, 'Stats5:')
    tiff = Cpt(TIFFPlugin, 'TIFF1:')


#cam_fs1 = StandardProsilica('XF:17IDA-BI:AMX{FS:1-Cam:1}', name='cam_fs1')
# cam_mono = StandardProsilica('XF:17IDA-BI:AMX{Mono:DCM-Cam:1}', name='cam_mono')
# comment out more unused cameras
# cam_fs2 = StandardProsilica('XF:17IDA-BI:AMX{FS:2-Cam:1}', name='cam_fs2')
# cam_fs3 = StandardProsilica('XF:17IDA-BI:AMX{FS:3-Cam:1}', name='cam_fs3')
# cam_fs4 = StandardProsilica('XF:17IDB-BI:AMX{FS:4-Cam:1}', name='cam_fs4')
cam_6 = StandardProsilica('XF:17IDB-ES:AMX{Cam:6}', name='cam_6')
cam_7 = StandardProsilica('XF:17IDB-ES:AMX{Cam:7}', name='cam_7')
xeye = StandardProsilica('XF:17IDB-ES:AMX{Cam:9}', name='xeye')

# all_standard_pros = [cam_fs1, cam_mono, cam_fs2, cam_fs3, cam_fs4, cam_6, cam_7, xeye]
all_standard_pros = [cam_fs2, cam_fs3, cam_fs4, cam_6, cam_7, xeye]
for camera in all_standard_pros:
    camera.read_attrs = ['stats1', 'stats2', 'stats3', 'stats4', 'stats5']
    camera.stats1.read_attrs = ['total', 'centroid']
    camera.stats2.read_attrs = ['total', 'centroid']
    camera.stats3.read_attrs = ['total', 'centroid']
    camera.stats4.read_attrs = ['total', 'centroid']
    camera.stats5.read_attrs = ['total', 'centroid']
    camera.tiff.read_attrs = []  # leaving just the 'image'

keithley = EpicsSignalRO('XF:17IDB-BI:AMX{Keith:1}readFloat', name='keithley')
