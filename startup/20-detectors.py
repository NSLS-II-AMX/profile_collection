from ophyd import (SingleTrigger, TIFFPlugin, ProsilicaDetector,
                   ImagePlugin, StatsPlugin, ROIPlugin, DetectorBase, HDF5Plugin,
                   AreaDetector)

import ophyd.areadetector.cam as cam

from ophyd.areadetector.filestore_mixins import (FileStoreTIFFIterativeWrite,
                                                 FileStoreHDF5IterativeWrite)

from ophyd import Component as Cpt

class StandardProsilica(SingleTrigger, ProsilicaDetector):
    #tiff = Cpt(TIFFPluginWithFileStore,
    #           suffix='TIFF1:',
    #           write_path_template='/XF16ID/data/')
    image = Cpt(ImagePlugin, 'image1:')
    roi1 = Cpt(ROIPlugin, 'ROI1:')
    roi2 = Cpt(ROIPlugin, 'ROI2:')
    roi3 = Cpt(ROIPlugin, 'ROI3:')
    roi4 = Cpt(ROIPlugin, 'ROI4:')
    stats1 = Cpt(StatsPlugin, 'Stats1:')
    stats2 = Cpt(StatsPlugin, 'Stats2:')
    stats3 = Cpt(StatsPlugin, 'Stats3:')
    stats4 = Cpt(StatsPlugin, 'Stats4:')

cam_fs1 = StandardProsilica('XF:17IDA-BI:AMX{FS:1-Cam:1}', name='cam_fs1')
cam_mono = StandardProsilica('XF:17IDA-BI:AMX{Mono:DCM-Cam:1}', name='cam_mono')

cam_fs2 = StandardProsilica('XF:17IDA-BI:AMX{FS:2-Cam:1}', name='cam_fs2')
cam_fs3 = StandardProsilica('XF:17IDA-BI:AMX{FS:3-Cam:1}', name='cam_fs3')
cam_fs4 = StandardProsilica('XF:17IDB-BI:AMX{FS:4-Cam:1}', name='cam_fs4')

cam_6 = StandardProsilica('XF:17IDB-ES:AMX{Cam:6}', name='cam_6')
cam_7 = StandardProsilica('XF:17IDB-ES:AMX{Cam:7}', name='cam_7')

all_standard_pros = [cam_fs1, cam_mono, cam_fs2, cam_fs3, cam_fs4, cam_6, cam_7]

for camera in all_standard_pros:
    camera.read_attrs = ['stats1', 'stats2','stats3','stats4']  #, 'tiff']
    #camera.tiff.read_attrs = []  # leaving just the 'image'
    camera.stats1.read_attrs = ['total', 'centroid']
    camera.stats2.read_attrs = ['total', 'centroid']
    camera.stats3.read_attrs = ['total', 'centroid']
    camera.stats4.read_attrs = ['total', 'centroid']
