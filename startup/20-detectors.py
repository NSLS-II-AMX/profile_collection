from ophyd import (SingleTrigger, TIFFPlugin, ProsilicaDetector,
                   ImagePlugin, StatsPlugin, ROIPlugin, DetectorBase, HDF5Plugin,
                   AreaDetector)

import ophyd.areadetector.cam as cam

from ophyd.areadetector.filestore_mixins import (FileStoreTIFFIterativeWrite,
                                                 FileStoreHDF5IterativeWrite)

from ophyd import Component as Cpt

class TIFFPluginWithFileStore(TIFFPlugin, FileStoreTIFFIterativeWrite):
    pass

class StandardProsilica(SingleTrigger, ProsilicaDetector):
    image = Cpt(ImagePlugin, 'image1:')
    roi1 = Cpt(ROIPlugin, 'ROI1:')
    stats1 = Cpt(StatsPlugin, 'Stats1:')
    stats5 = Cpt(StatsPlugin, 'Stats5:')

class StandardProsilicaWithTIFF(StandardProsilica):
    tiff = Cpt(TIFFPluginWithFileStore,
               suffix='TIFF1:',
               write_path_template='/tmp/')

cam_fs1 = StandardProsilicaWithTIFF('XF:17IDA-BI:AMX{FS:1-Cam:1}', name='cam_fs1')
cam_mono = StandardProsilicaWithTIFF('XF:17IDA-BI:AMX{Mono:DCM-Cam:1}', name='cam_mono')

cam_fs2 = StandardProsilicaWithTIFF('XF:17IDA-BI:AMX{FS:2-Cam:1}', name='cam_fs2')
cam_fs3 = StandardProsilicaWithTIFF('XF:17IDA-BI:AMX{FS:3-Cam:1}', name='cam_fs3')
cam_fs4 = StandardProsilicaWithTIFF('XF:17IDB-BI:AMX{FS:4-Cam:1}', name='cam_fs4')

cam_6 = StandardProsilicaWithTIFF('XF:17IDB-ES:AMX{Cam:6}', name='cam_6')
cam_7 = StandardProsilicaWithTIFF('XF:17IDB-ES:AMX{Cam:7}', name='cam_7')

all_standard_pros = [cam_fs1, cam_mono, cam_fs2, cam_fs3, cam_fs4, cam_6, cam_7]

for camera in all_standard_pros:
    camera.read_attrs = ['stats1', 'stats5', 'tiff']
    camera.stats1.read_attrs = ['total', 'centroid']
    camera.stats5.read_attrs = ['total', 'centroid']
    camera.tiff.read_attrs = []  # leaving just the 'image'

keithley = EpicsSignalRO('XF:17IDB-BI:AMX{Keith:1}readFloat', name='keithley')
