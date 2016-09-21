/*	$NetBSD: tdvfbreg.h,v 1.3 2012/07/20 21:31:28 rkujawa Exp $	*/

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

#ifndef MNTVA2000REG_H
#define MNTVA2000REG_H

//#define TDV_INITENABLE_EN_INIT  __BIT(0)      

/* address space */
#define MNTVA_OFF_REG		0x5f0000	/* memory mapped registers */
#define MNTVA_OFF_FB		0x000000	/* frame buffer */
#define MNTVA_FB_SIZE		0x5f0000

#endif /* MNTVA2000REG_H */
