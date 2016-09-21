/*  $NetBSD$  */

/*
 * Copyright (c) 2012 The NetBSD Foundation, Inc.   
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Radoslaw Kujawa.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0,
    "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/endian.h>
#include <sys/bus.h>

#include <amiga/amiga/device.h>
#include <amiga/amiga/isr.h>

#include <amiga/dev/zbusvar.h>
#include <amiga/dev/mntvavar.h>
#include <amiga/dev/mntvareg.h>

#include <dev/videomode/videomode.h>

#include "opt_wsemul.h"
//#include "opt_mntva.h"

static int mntva_match(device_t, cfdata_t, void *);
static void mntva_attach(device_t, device_t, void *);

//static uint32_t mntva_cvg_read(struct mntva_softc *sc, uint32_t reg);
static void mntva_write(struct mntva_softc *sc, uint32_t reg, uint32_t val);

static bool mntva_videomode_set(struct mntva_softc *sc);

static paddr_t mntva_mmap(void *v, void *vs, off_t offset, int prot);
static int mntva_ioctl(void *v, void *vs, u_long cmd, void *data, int flag,
    struct lwp *l);
static void mntva_init_screen(void *cookie, struct vcons_screen *scr,
    int existing, long *defattr);
//static void mntva_init_palette(struct mntva_softc *sc);
/* blitter support */
static void mntva_rectfill(struct mntva_softc *sc, int x, int y, int wi,
    int he, uint32_t color);
/*static void mntva_bitblt(struct mntva_softc *sc, int xs, int ys, int xd, 
int yd, int wi, int he);*/

/* accelerated raster ops */
/*static void mntva_eraserows(void *cookie, int row, int nrows, 
        long fillattr);
static void mntva_copyrows(void *cookie, int srcrow, int dstrow, int nrows);*/

CFATTACH_DECL_NEW(mntva, sizeof(struct mntva_softc),
    mntva_match, mntva_attach, NULL, NULL);

struct wsdisplay_accessops mntva_accessops = {
	mntva_ioctl,
	mntva_mmap,
	NULL,			// alloc_screen 
	NULL,			// free_screen 
	NULL,			// show_screen 
	NULL,			// load_font 
	NULL,			// pollc 
	NULL			// scroll 
};

static int
mntva_match(device_t parent, cfdata_t match, void *aux)
{
	struct zbus_args *zap = aux;

	if (zap->manid == 0x6d6e && zap->prodid == 1) {
		aprint_normal("mntva_match... success!\n");
		return (1);
	}

	aprint_normal("mntva_match... fail\n");
	return (0);
}

#define MNTVA_COLORMODE 0x48
#define MNTVA_SCALEMODE 0x04
#define MNTVA_SCREENW 0x06
#define MNTVA_SCREENH 0x08

static void
mntva_attach(device_t parent, device_t self, void *aux)
{
	struct mntva_softc *sc = device_private(self);
	struct wsemuldisplaydev_attach_args ws_aa;
	struct rasops_info *ri;
	long defattr;

	struct zbus_args *zap = aux;

	sc->sc_dev = self;
	sc->sc_fba = (bus_addr_t) zap->va;
	sc->sc_rga = (bus_addr_t) zap->va + MNTVA_OFF_REG;
	sc->sc_memsize = MNTVA_FB_SIZE;

	sc->sc_bst.base = (bus_addr_t) zap->va;
	sc->sc_bst.absm = &amiga_bus_stride_1;
	sc->sc_iot = &sc->sc_bst;

	aprint_normal_dev(sc->sc_dev, "registers at 0x%08x, fb at 0x%08x\n",
	    (uint32_t) sc->sc_rga, (uint32_t) sc->sc_fba);
	aprint_normal_dev(sc->sc_dev, "%zu MB framebuffer memory present\n",
	    sc->sc_memsize / 1024 / 1024);

	if (bus_space_map(sc->sc_iot, 0, sc->sc_memsize + 0x1000, 0,
		&sc->sc_ioh)) {
		aprint_error_dev(sc->sc_dev, "bus_space_map failed\n");
		return;
	}

	/*if (bus_space_subregion(sc->sc_cvgt, sc->sc_cvgh, 0, 
	 * MNTVA_FB_SIZE, &sc->sc_fbh)) {
	 * aprint_error_dev(sc->sc_dev, "unable to map the framebuffer");  
	 * } */

	sc->sc_width = 1280;
	sc->sc_height = 720;
	sc->sc_bpp = 16;
	sc->sc_linebytes = 4096;

	aprint_normal_dev(sc->sc_dev, "setting %dx%d %d bpp resolution\n",
	    sc->sc_width, sc->sc_height, sc->sc_bpp);

	sc->sc_videomode = pick_mode_by_ref(sc->sc_width, sc->sc_height, 60);
	mntva_videomode_set(sc);

	sc->sc_defaultscreen_descr = (struct wsscreen_descr) {
	"default",
		    0, 0,
		    NULL, 8, 16, WSSCREEN_WSCOLORS | WSSCREEN_HILIT, NULL};
	sc->sc_screens[0] = &sc->sc_defaultscreen_descr;
	sc->sc_screenlist = (struct wsscreen_list) {
	1, sc->sc_screens};
	sc->sc_mode = WSDISPLAYIO_MODE_EMUL;

	vcons_init(&sc->vd, sc, &sc->sc_defaultscreen_descr, &mntva_accessops);
	sc->vd.init_screen = mntva_init_screen;

	ri = &sc->sc_console_screen.scr_ri;

	if (sc->sc_console_screen.scr_ri.ri_rows == 0) {
		vcons_init_screen(&sc->vd, &sc->sc_console_screen, 1,
		    &defattr);
	} else
		(*ri->ri_ops.allocattr) (ri, 0, 0, 0, &defattr);

	ws_aa.console = false;
	ws_aa.scrdata = &sc->sc_screenlist;
	ws_aa.accessops = &mntva_accessops;
	ws_aa.accesscookie = &sc->vd;

	config_found(sc->sc_dev, &ws_aa, wsemuldisplaydevprint);
}

/*static void
mntva_init_palette(struct mntva_softc *sc)
{
  //int i, j;

  j = 0;
  for (i = 0; i < 256; i++) {
    sc->sc_cmap_red[i] = rasops_cmap[j];
    sc->sc_cmap_green[i] = rasops_cmap[j + 1];
    sc->sc_cmap_blue[i] = rasops_cmap[j + 2];
    j += 3;
    }
}*/

static void
mntva_init_screen(void *cookie, struct vcons_screen *scr, int existing,
    long *defattr)
{
	struct mntva_softc *sc = cookie;
	struct rasops_info *ri = &scr->scr_ri;

	wsfont_init();

	ri->ri_depth = sc->sc_bpp;
	ri->ri_width = sc->sc_width;
	ri->ri_height = sc->sc_height;
	ri->ri_stride = sc->sc_linebytes;
	ri->ri_flg = RI_CENTER;

#if BYTE_ORDER == BIG_ENDIAN
#if 0
	if (sc->sc_bpp == 16)
		ri->ri_flg |= RI_BITSWAP;
#endif
#endif

	ri->ri_bits = (char *) bus_space_vaddr(sc->sc_iot, 0);
	aprint_normal_dev(sc->sc_dev, "ri_bits: %p\n", ri->ri_bits);

	scr->scr_flags |= VCONS_DONT_READ;

	rasops_init(ri, 0, 0);
	ri->ri_caps = WSSCREEN_WSCOLORS;
	rasops_reconfig(ri, sc->sc_height / ri->ri_font->fontheight,
	    sc->sc_width / ri->ri_font->fontwidth);

	ri->ri_hw = scr;

	//ri->ri_ops.eraserows = mntva_eraserows;
	//ri->ri_ops.copyrows = mntva_copyrows;
}

static bool
mntva_videomode_set(struct mntva_softc *sc)
{
	mntva_write(sc, MNTVA_SCALEMODE, 0);
	mntva_write(sc, MNTVA_SCREENW, sc->sc_width);
	mntva_write(sc, MNTVA_SCREENH, sc->sc_height);

	if (sc->sc_bpp == 16)
		mntva_write(sc, MNTVA_COLORMODE, 1);
	else if (sc->sc_bpp == 32)
		mntva_write(sc, MNTVA_COLORMODE, 2);

	mntva_rectfill(sc, 0, 0, sc->sc_width, sc->sc_height, 0xffffffff);

	return true;
}

/*static uint32_t
mntva_cvg_read(struct mntva_softc *sc, uint32_t reg) 
{
  uint32_t rv;
  rv = bus_space_read_2(sc->sc_cvgt, sc->sc_cvgh, reg);
  return rv;
}*/

static void
mntva_write(struct mntva_softc *sc, uint32_t reg, uint32_t val)
{
	aprint_normal("write val %x to reg %x\n", val, reg);
	bus_space_write_2(sc->sc_iot, sc->sc_ioh, MNTVA_OFF_REG + reg, val);
}

static void
mntva_rectfill(struct mntva_softc *sc, int x, int y, int wi, int he,
    uint32_t color)
{
	mntva_write(sc, MNTVA_COLORMODE, 2);
	mntva_write(sc, 0x28, (uint16_t) color);
	mntva_write(sc, 0x20, (uint16_t) x);
	mntva_write(sc, 0x22, (uint16_t) y);
	mntva_write(sc, 0x24, (uint16_t) x + wi - 1);
	mntva_write(sc, 0x26, (uint16_t) y + he - 1);
	mntva_write(sc, 0x2a, 1);
}

/*
static void
mntva_bitblt(struct mntva_softc *sc, int xs, int ys, int xd, int yd, int wi,
    int he) 
{
}*/
/*
static void
mntva_copyrows(void *cookie, int srcrow, int dstrow, int nrows)
{
  struct mntva_softc *sc;
  struct rasops_info *ri;
  struct vcons_screen *scr;
  int x, ys, yd, wi, he;

  ri = cookie;
  scr = ri->ri_hw;
  sc = scr->scr_cookie;

  if (sc->sc_mode == WSDISPLAYIO_MODE_EMUL) {
    x = ri->ri_xorigin;
    ys = ri->ri_yorigin + ri->ri_font->fontheight * srcrow;
    yd = ri->ri_yorigin + ri->ri_font->fontheight * dstrow;
    wi = ri->ri_emuwidth;
    he = ri->ri_font->fontheight * nrows;
    mntva_bitblt(sc, x, ys, x, yd, wi, he);
  }
}

static void
mntva_eraserows(void *cookie, int row, int nrows, long fillattr)
{

  struct mntva_softc *sc;
  struct rasops_info *ri;
  struct vcons_screen *scr;
  int x, y, wi, he, fg, bg, ul;

  ri = cookie;
  scr = ri->ri_hw;
  sc = scr->scr_cookie;

  if (sc->sc_mode == WSDISPLAYIO_MODE_EMUL) {
    rasops_unpack_attr(fillattr, &fg, &bg, &ul);
    if ((row == 0) && (nrows == ri->ri_rows)) 
      mntva_rectfill(sc, 0, 0, ri->ri_width,
          ri->ri_height, ri->ri_devcmap[bg]);
    else {
      x = ri->ri_xorigin;
      y = ri->ri_yorigin + ri->ri_font->fontheight * row;
      wi = ri->ri_emuwidth;
      he = ri->ri_font->fontheight * nrows;
      mntva_rectfill(sc, x, y, wi, he, ri->ri_devcmap[bg]);
    }
  }
  }*/

static int
mntva_ioctl(void *v, void *vs, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct vcons_data *vd;
	struct mntva_softc *sc;
	struct wsdisplay_fbinfo *wsfbi;
	struct vcons_screen *ms;

	vd = v;
	sc = vd->cookie;
	ms = vd->active;

	switch (cmd) {
	case WSDISPLAYIO_GTYPE:
		*(u_int *) data = WSDISPLAY_TYPE_UNKNOWN;
		return 0;

		/*case WSDISPLAYIO_GET_BUSID:
		 * return wsdisplayio_busid_pci(sc->sc_dev, sc->sc_pc,
		 * sc->sc_pcitag, data); */

	case WSDISPLAYIO_GINFO:
		if (ms == NULL)
			return ENODEV;

		wsfbi = (void *) data;
		wsfbi->height = ms->scr_ri.ri_height;
		wsfbi->width = ms->scr_ri.ri_width;
		wsfbi->depth = ms->scr_ri.ri_depth;
		wsfbi->cmsize = 256;
		return 0;

	case WSDISPLAYIO_LINEBYTES:
		*(u_int *) data = sc->sc_linebytes;
		return 0;

	case WSDISPLAYIO_SMODE:
		{
			int new_mode = *(int *) data;
			if (new_mode != sc->sc_mode) {
				sc->sc_mode = new_mode;
				if (new_mode == WSDISPLAYIO_MODE_EMUL)
					vcons_redraw_screen(ms);
			}
			return 0;
		}
	case WSDISPLAYIO_GET_FBINFO:
		{
			struct wsdisplayio_fbinfo *fbi = data;
			struct rasops_info *ri;
			int ret;

			ri = &sc->vd.active->scr_ri;
			ret = wsdisplayio_get_fbinfo(ri, fbi);
			return ret;
		}
	}
	return EPASSTHROUGH;
}

static paddr_t
mntva_mmap(void *v, void *vs, off_t offset, int prot)
{
	struct vcons_data *vd;
	struct mntva_softc *sc;
	paddr_t pa;

	vd = v;
	sc = vd->cookie;

	if (offset < sc->sc_memsize) {
		pa = bus_space_mmap(sc->sc_iot, offset, 0, prot,
		    BUS_SPACE_MAP_LINEAR);
		return pa;
	}

	return -1;
}
