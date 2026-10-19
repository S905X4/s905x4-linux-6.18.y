=============================================
drm/meson AmLogic Meson Video Processing Unit
=============================================

.. kernel-doc:: drivers/gpu/drm/meson/meson_drv.c
   :doc: Video Processing Unit

Video Processing Unit
=====================

The Amlogic Meson Display controller is composed of several components
that are going to be documented below:

.. code::

  DMC|---------------VPU (Video Processing Unit)----------------|------HHI------|
     | vd1   _______     _____________    _________________     |               |
  D  |-------|      |----|            |   |                |    |   HDMI PLL    |
  D  | vd2   | VIU  |    | Video Post |   | Video Encoders |<---|-----VCLK      |
  R  |-------|      |----| Processing |   |                |    |               |
     | osd2  |      |    |            |---| Enci ----------|----|-----VDAC------|
  R  |-------| CSC  |----| Scalers    |   | Encp ----------|----|----HDMI-TX----|
  A  | osd1  |      |    | Blenders   |   | Encl ----------|----|---------------|
  M  |-------|______|----|____________|   |________________|    |               |
  ___|__________________________________________________________|_______________|

Video Input Unit
================

.. kernel-doc:: drivers/gpu/drm/meson/meson_viu.c
   :doc: Video Input Unit

Video Post Processing
=====================

.. kernel-doc:: drivers/gpu/drm/meson/meson_vpp.c
   :doc: Video Post Processing

Video Encoder
=============

.. kernel-doc:: drivers/gpu/drm/meson/meson_venc.c
   :doc: Video Encoder

Video Clocks
============

.. kernel-doc:: drivers/gpu/drm/meson/meson_vclk.c
   :doc: Video Clocks

HDMI Video Output
=================

.. kernel-doc:: drivers/gpu/drm/meson/meson_dw_hdmi.c
   :doc: HDMI Output

SC2 protected HDMI registers
----------------------------

SC2 firmware protects the DWC clock-disable, HDCP configuration and HDCP2.2
control registers. Direct byte MMIO reads are possible, but writes to those
registers are ignored. The SC2 register callbacks use the firmware HDMI
register services (SMC 0x82000018/0x82000019) with the encoded secure DWC
selector, not the physical MMIO address. Other DWC registers and other
Meson variants retain their existing access paths.

The unencrypted bridge also initializes HDCP2.2 control to the inactive
setting used by the SC2 CoreELEC reference driver, without HDCP authentication.
On HK1 RBOX X4, a reversible probe changed HDCPCFG1 from 0x01 to 0x03 through
SMC, while a direct MMIO write to that same register was ignored. Correct
register readback, measured pixel clocks and active DRM scanout are not proof
that a physical sink displays the intended pixels.
