/*	$NetBSD$	*/

/*
 * Copyright (c) 2012, 2016 The NetBSD Foundation, Inc.		
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lukas F. Hartmann.
 * This code is derived from software contributed to The NetBSD Foundation
 * by Radoslaw Kujawa.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *		notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *		notice, this list of conditions and the following disclaimer in the
 *		documentation and/or other materials provided with the distribution.
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
#include <sys/cpu.h>

#include <amiga/amiga/device.h>
#include <amiga/amiga/isr.h>

#include <amiga/dev/zbusvar.h>
#include <amiga/dev/mntvavar.h>
#include <amiga/dev/mntvareg.h>

#include "opt_wsemul.h"
//#include "opt_mntva.h"

#include <dev/wsfb/genfbvar.h>
#include "opt_wsfb.h"

static int mntva_match(device_t, cfdata_t, void *);
static void mntva_attach(device_t, device_t, void *);

static uint16_t mntva_reg_read(struct mntva_softc *sc, uint32_t reg);
static void mntva_reg_write(struct mntva_softc *sc, uint32_t reg, uint32_t val);

static bool mntva_mode_set(struct mntva_softc *sc);

static paddr_t mntva_mmap(void *v, void *vs, off_t offset, int prot);
static int mntva_ioctl(void *v, void *vs, u_long cmd, void *data, int flag,
		struct lwp *l);
static void mntva_init_screen(void *cookie, struct vcons_screen *scr,
		int existing, long *defattr);
static void mntva_init_palette(struct mntva_softc *sc);
/* blitter support */
static void mntva_rectfill(struct mntva_softc *sc, int x, int y, int wi,
		int he, uint32_t color);
static void mntva_bitblt(struct mntva_softc *sc, int xs, int ys, int xd, 
int yd, int wi, int he);

/* accelerated raster ops */
static void mntva_eraserows(void *cookie, int row, int nrows, 
				long fillattr);
static void mntva_copyrows(void *cookie, int srcrow, int dstrow, int nrows);

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
		return 1;
	}

	return 0;
}

static void
mntva_attach(device_t parent, device_t self, void *aux)
{
	struct mntva_softc *sc = device_private(self);
	struct wsemuldisplaydev_attach_args ws_aa;
	struct rasops_info *ri;
	long defattr;

	struct zbus_args *zap = aux;

	sc->sc_dev = self;
	sc->sc_memsize = MNTVA_FB_SIZE;

	sc->sc_bst.base = (bus_addr_t) zap->va;
	sc->sc_bst.absm = &amiga_bus_stride_1;
	sc->sc_iot = &sc->sc_bst;

	if (bus_space_map(sc->sc_iot, MNTVA_OFF_FB, sc->sc_memsize + 0x1000, 0,
		&sc->sc_fbh)) {
		aprint_error_dev(sc->sc_dev, "mapping framebuffer failed\n");
		return;
	}
	if (bus_space_map(sc->sc_iot, MNTVA_OFF_REG, MNTVA_REG_SIZE , 0,
		&sc->sc_regh)) {
		aprint_error_dev(sc->sc_dev, "mapping registers failed\n");
		return;
	}

	sc->sc_regpa = (bus_addr_t) kvtop((void*) sc->sc_regh);
	sc->sc_fbpa = (bus_addr_t) kvtop((void*) sc->sc_fbh);

	/* print the physical and virt addresses for registers and fb */
	aprint_normal_dev(sc->sc_dev, 
			"registers at pa/va 0x%08x/0x%08x, fb at pa/va 0x%08x/0x%08x\n",
			(uint32_t) sc->sc_regpa,
			(uint32_t) bus_space_vaddr(sc->sc_iot, sc->sc_regh), 
			(uint32_t) sc->sc_fbpa,
			(uint32_t) bus_space_vaddr(sc->sc_iot, sc->sc_fbh));

	sc->sc_width = 1280;
	sc->sc_height = 720;
	sc->sc_bpp = 16;
	sc->sc_linebytes = 4096;

	aprint_normal_dev(sc->sc_dev, "%zu kB framebuffer memory present\n",
			sc->sc_memsize / 1024);

	aprint_normal_dev(sc->sc_dev, "setting %dx%d %d bpp resolution\n",
			sc->sc_width, sc->sc_height, sc->sc_bpp);

	mntva_mode_set(sc);

	sc->sc_defaultscreen_descr = (struct wsscreen_descr) {
			"default", 0, 0, NULL, 8, 16, 
			WSSCREEN_WSCOLORS | WSSCREEN_HILIT, NULL };
	sc->sc_screens[0] = &sc->sc_defaultscreen_descr;
	sc->sc_screenlist = (struct wsscreen_list) { 1, sc->sc_screens };
	sc->sc_mode = WSDISPLAYIO_MODE_EMUL;

	vcons_init(&sc->vd, sc, &sc->sc_defaultscreen_descr, &mntva_accessops);
	sc->vd.init_screen = mntva_init_screen;

	ri = &sc->sc_console_screen.scr_ri;

	mntva_init_palette(sc);
	
	/*vcons_init_screen(&sc->vd, &sc->sc_console_screen, 1,
										&defattr);

	sc->sc_console_screen.scr_flags |= VCONS_SCREEN_IS_STATIC; 
	vcons_redraw_screen(&sc->sc_console_screen);

	sc->sc_defaultscreen_descr.textops = &ri->ri_ops;
	sc->sc_defaultscreen_descr.capabilities = ri->ri_caps;
	sc->sc_defaultscreen_descr.nrows = ri->ri_rows;
	sc->sc_defaultscreen_descr.ncols = ri->ri_cols;

	wsdisplay_cnattach(&sc->sc_defaultscreen_descr, ri, 0, 0,
										 defattr);
										 vcons_replay_msgbuf(&sc->sc_console_screen);*/

	if (sc->sc_console_screen.scr_ri.ri_rows == 0) {
		vcons_init_screen(&sc->vd, &sc->sc_console_screen, 1, &defattr);
	} else
	(*ri->ri_ops.allocattr) (ri, 0, 0, 0, &defattr);

	ws_aa.console = false;
	ws_aa.scrdata = &sc->sc_screenlist;
	ws_aa.accessops = &mntva_accessops;
	ws_aa.accesscookie = &sc->vd;

	//config_found(sc->sc_dev, &ws_aa, wsemuldisplaydevprint);
	
	config_found_ia(sc->sc_dev, "wsemuldisplaydev", &ws_aa,
			wsemuldisplaydevprint);
}

static void
mntva_init_palette(struct mntva_softc *sc)
{
	int i, j;

	j = 0;
	for (i=0; i<256; i++) {
		mntva_reg_write(sc, 0x200+i*2, rasops_cmap[j]);
		mntva_reg_write(sc, 0x400+i*2, rasops_cmap[j+1]);
		mntva_reg_write(sc, 0x600+i*2, rasops_cmap[j+2]);
		j+=3;
	}
}

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
	ri->ri_flg = 0; //RI_CENTER;

	//ri->ri_flg = RI_BSWAP;

	ri->ri_bits = (char *) bus_space_vaddr(sc->sc_iot, sc->sc_fbh);
	aprint_normal_dev(sc->sc_dev, "ri_bits: %p\n", ri->ri_bits);

	scr->scr_flags |= VCONS_DONT_READ;

	rasops_init(ri, 0, 0);
	ri->ri_caps = WSSCREEN_WSCOLORS;
	rasops_reconfig(ri, sc->sc_height / ri->ri_font->fontheight,
			sc->sc_width / ri->ri_font->fontwidth);

	ri->ri_hw = scr;

	ri->ri_ops.eraserows = mntva_eraserows;
	ri->ri_ops.copyrows = mntva_copyrows;
}

static bool
mntva_mode_set(struct mntva_softc *sc)
{
	mntva_reg_write(sc, MNTVA_SCALEMODE, 0);
	mntva_reg_write(sc, MNTVA_SCREENW, sc->sc_width);
	mntva_reg_write(sc, MNTVA_SCREENH, sc->sc_height);

	if (sc->sc_bpp == 8)
		mntva_reg_write(sc, MNTVA_COLORMODE, MNTVA_COLORMODE8);
	else if (sc->sc_bpp == 16)
		mntva_reg_write(sc, MNTVA_COLORMODE, MNTVA_COLORMODE16);
	else if (sc->sc_bpp == 32)
		mntva_reg_write(sc, MNTVA_COLORMODE, MNTVA_COLORMODE32);

	mntva_reg_write(sc, MNTVA_PANPTRHI, 0);
	mntva_reg_write(sc, MNTVA_PANPTRLO, 0);
	mntva_reg_write(sc, MNTVA_BLITTERBASEHI, 0);
	mntva_reg_write(sc, MNTVA_BLITTERBASELO, 0);
	
	mntva_rectfill(sc, 0, 0, sc->sc_width, sc->sc_height, 0xffffffff);

	return true;
}

static uint16_t
mntva_reg_read(struct mntva_softc *sc, uint32_t reg) 
{
	uint32_t rv;
	rv = bus_space_read_2(sc->sc_iot, sc->sc_regh, reg);
	return rv;
}

static void
mntva_reg_write(struct mntva_softc *sc, uint32_t reg, uint32_t val)
{
	//aprint_normal("write val %x to reg %x\n", val, reg);
	bus_space_write_2(sc->sc_iot, sc->sc_regh, reg, val);
}

static void
mntva_rectfill(struct mntva_softc *sc, int x, int y, int wi, int he,
		uint32_t color)
{
	mntva_reg_write(sc, MNTVA_BLITTERRGB, (uint16_t) color);
	mntva_reg_write(sc, MNTVA_BLITTERX1, (uint16_t) x);
	mntva_reg_write(sc, MNTVA_BLITTERY1, (uint16_t) y);
	mntva_reg_write(sc, MNTVA_BLITTERX2, (uint16_t) x + wi - 1);
	mntva_reg_write(sc, MNTVA_BLITTERY2, (uint16_t) y + he - 1);
	mntva_reg_write(sc, MNTVA_BLITTER_ENABLE, MNTVA_BLITTER_FILL);

	while(mntva_reg_read(sc, MNTVA_BLITTER_ENABLE)) {
		// busy wait
	}
}

static void
mntva_bitblt(struct mntva_softc *sc, int xs, int ys, int xd, int yd, int wi,
		int he)
{
	mntva_reg_write(sc, MNTVA_BLITTERX1, (uint16_t) xd);
	mntva_reg_write(sc, MNTVA_BLITTERY1, (uint16_t) yd);
	mntva_reg_write(sc, MNTVA_BLITTERX2, (uint16_t) xd + wi - 1);
	mntva_reg_write(sc, MNTVA_BLITTERY2, (uint16_t) yd + he - 1);
	mntva_reg_write(sc, MNTVA_BLITTERX3, (uint16_t) xs);
	mntva_reg_write(sc, MNTVA_BLITTERY3, (uint16_t) ys);
	mntva_reg_write(sc, MNTVA_BLITTERX4, (uint16_t) xs + wi - 1);
	mntva_reg_write(sc, MNTVA_BLITTERY4, (uint16_t) ys + he - 1);
	mntva_reg_write(sc, MNTVA_BLITTER_ENABLE, MNTVA_BLITTER_COPY);
	
	while(mntva_reg_read(sc, MNTVA_BLITTER_ENABLE)) {
		// busy wait
	}
}

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
}

static int
mntva_ioctl(void *v, void *vs, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct vcons_data *vd;
	struct mntva_softc *sc;
	struct wsdisplay_fbinfo *wsfbi;
	struct vcons_screen *ms;
	struct wsdisplayio_bus_id *busid;

	vd = v;
	sc = vd->cookie;
	ms = vd->active;
	
	//aprint_normal_dev(sc->sc_dev, "mntva_ioctl cmd 0x%08x...",(uint32_t)cmd);

	switch (cmd) {
	case WSDISPLAYIO_GTYPE:
		//aprint_normal_dev(sc->sc_dev, "...WSDISPLAYIO_GTYPE");
		*(u_int *) data = WSDISPLAY_TYPE_UNKNOWN;
		return 0;

	case WSDISPLAYIO_GET_BUSID:
		//aprint_normal_dev(sc->sc_dev, "...WSDISPLAYIO_GET_BUSID");
		busid = data;
		busid->bus_type = WSDISPLAYIO_BUS_SOC;
		return 0;

	case WSDISPLAYIO_GINFO:
		//aprint_normal_dev(sc->sc_dev, "...WSDISPLAYIO_GINFO %p",ms);
		if (ms == NULL)
			return ENODEV;

		wsfbi = (void *) data;
		wsfbi->height = ms->scr_ri.ri_height;
		wsfbi->width = ms->scr_ri.ri_width;
		wsfbi->depth = ms->scr_ri.ri_depth;
		wsfbi->cmsize = 256;
		return 0;

	case WSDISPLAYIO_LINEBYTES:
		//aprint_normal_dev(sc->sc_dev, "...WSDISPLAYIO_LINEBYTES");
		*(u_int *) data = sc->sc_linebytes;
		return 0;

	case WSDISPLAYIO_SMODE:
		{
			int new_mode = *(int *) data;
			//aprint_normal_dev(sc->sc_dev, "...WSDISPLAYIO_SMODE %d",new_mode);
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

			//aprint_normal_dev(sc->sc_dev, "...WSDISPLAYIO_GET_FBINFO");
			ri = &sc->vd.active->scr_ri;
			ret = wsdisplayio_get_fbinfo(ri, fbi);
			return ret;
		}
	}
	//aprint_normal_dev(sc->sc_dev, "...EPASSTHROUGH");
	
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

	//aprint_normal_dev(sc->sc_dev, "mntva_mmap offset 0x%08x...",(uint32_t)offset);

	if (offset < sc->sc_memsize) {
		pa = bus_space_mmap(sc->sc_iot, sc->sc_fbh + offset, 0, prot,
				BUS_SPACE_MAP_LINEAR);
		//aprint_normal_dev(sc->sc_dev, "... pa: 0x%08x...",(uint32_t)pa);
		return pa;
	}

	return -1;
}
