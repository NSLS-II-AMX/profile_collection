print(f"Loading {__file__}")

from ophyd.mca import (EpicsMCA, EpicsDXP, Mercury1, SoftDXPTrigger)

class AMXMercury(Mercury1, SoftDXPTrigger):
    @property
    def hints(self):
        return {'fields': [self.mca.rois.roi0.count.name]}

mercury = AMXMercury('XF:17IDB-ES:AMX{Det:Mer}', name='mercury')
mercury.read_attrs = ['mca.spectrum', 'mca.preset_live_time', 'mca.rois.roi0.count',
                      'mca.rois.roi1.count', 'mca.rois.roi2.count', 'mca.rois.roi3.count']
