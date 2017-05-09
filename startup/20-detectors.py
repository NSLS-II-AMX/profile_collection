from ophyd import (SingleTrigger, TIFFPlugin, ProsilicaDetector,
                   ImagePlugin, StatsPlugin, ROIPlugin, DetectorBase, HDF5Plugin,
                   TransformPlugin, ProcessPlugin, AreaDetector)

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


class StandardProsilicaWithTIFF(StandardProsilica):
    tiff = Cpt(TIFFPluginWithFileStore,
               suffix='TIFF1:',
               write_path_template='/tmp/',
               root='/tmp/')

cam_fs1 = StandardProsilica('XF:17IDA-BI:AMX{FS:1-Cam:1}', name='cam_fs1')
cam_mono = StandardProsilica('XF:17IDA-BI:AMX{Mono:DCM-Cam:1}', name='cam_mono')
cam_fs2 = StandardProsilica('XF:17IDA-BI:AMX{FS:2-Cam:1}', name='cam_fs2')
cam_fs3 = StandardProsilica('XF:17IDA-BI:AMX{FS:3-Cam:1}', name='cam_fs3')
cam_fs4 = StandardProsilica('XF:17IDB-BI:AMX{FS:4-Cam:1}', name='cam_fs4')
cam_6 = StandardProsilica('XF:17IDB-ES:AMX{Cam:6}', name='cam_6')
cam_7 = StandardProsilica('XF:17IDB-ES:AMX{Cam:7}', name='cam_7')
xeye = StandardProsilica('XF:17IDB-ES:AMX{Cam:9}', name='xeye')

cam_fs1_tiff = StandardProsilicaWithTIFF('XF:17IDA-BI:AMX{FS:1-Cam:1}', name='cam_fs1_tiff')
cam_mono_tiff = StandardProsilicaWithTIFF('XF:17IDA-BI:AMX{Mono:DCM-Cam:1}', name='cam_mono_tiff')
cam_fs2_tiff = StandardProsilicaWithTIFF('XF:17IDA-BI:AMX{FS:2-Cam:1}', name='cam_fs2_tiff')
cam_fs3_tiff = StandardProsilicaWithTIFF('XF:17IDA-BI:AMX{FS:3-Cam:1}', name='cam_fs3_tiff')
cam_fs4_tiff = StandardProsilicaWithTIFF('XF:17IDB-BI:AMX{FS:4-Cam:1}', name='cam_fs4_tiff')
cam_6_tiff = StandardProsilicaWithTIFF('XF:17IDB-ES:AMX{Cam:6}', name='cam_6_tiff')
cam_7_tiff = StandardProsilicaWithTIFF('XF:17IDB-ES:AMX{Cam:7}', name='cam_7_tiff')

all_standard_pros = [cam_fs1, cam_mono, cam_fs2, cam_fs3, cam_fs4, cam_6, cam_7, xeye]
for camera in all_standard_pros:
    camera.read_attrs = ['stats1', 'stats2', 'stats3', 'stats4', 'stats5']
    camera.stats1.read_attrs = ['total', 'centroid']
    camera.stats2.read_attrs = ['total', 'centroid']
    camera.stats3.read_attrs = ['total', 'centroid']
    camera.stats4.read_attrs = ['total', 'centroid']
    camera.stats5.read_attrs = ['total', 'centroid']

all_standard_pros_tiff = [cam_fs1_tiff, cam_mono_tiff, cam_fs2_tiff, cam_fs3_tiff, cam_fs4_tiff, cam_6_tiff, cam_7_tiff]
for camera in all_standard_pros_tiff:
    camera.read_attrs = ['stats1', 'stats2', 'stats3', 'stats4', 'stats5', 'tiff']
    camera.stats1.read_attrs = ['total', 'centroid']
    camera.stats2.read_attrs = ['total', 'centroid']
    camera.stats3.read_attrs = ['total', 'centroid']
    camera.stats4.read_attrs = ['total', 'centroid']
    camera.stats5.read_attrs = ['total', 'centroid']
    camera.tiff.read_attrs = []  # leaving just the 'image'

keithley = EpicsSignalRO('XF:17IDB-BI:AMX{Keith:1}readFloat', name='keithley')
