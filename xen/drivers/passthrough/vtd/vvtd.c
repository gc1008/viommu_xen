/*
 * vvtd.c
 *
 * virtualize VTD for HVM.
 *
 * Copyright (C) 2017 Chao Gao, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms and conditions of the GNU General Public
 * License, version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#include <xen/sched.h>
#include <xen/types.h>
#include <xen/viommu.h>
#include <xen/xmalloc.h>
#include <asm/current.h>
#include <asm/hvm/domain.h>

#include "iommu.h"

/* Supported capabilities by vvtd */
#define VVTD_MAX_CAPS VIOMMU_CAP_IRQ_REMAPPING

#define VVTD_FRCD_NUM   1ULL
#define VVTD_FRCD_START (DMAR_IRTA_REG + 8)
#define VVTD_FRCD_END   (VVTD_FRCD_START + VVTD_FRCD_NUM * 16)
#define VVTD_MAX_OFFSET VVTD_FRCD_END

struct hvm_hw_vvtd {
    uint32_t regs[VVTD_MAX_OFFSET/sizeof(uint32_t)];
};

struct vvtd {
    /* Base address of remapping hardware register-set */
    uint64_t base_addr;
    /* Point back to the owner domain */
    struct domain *domain;

    struct hvm_hw_vvtd hw;
};

/* Setting viommu_verbose enables debugging messages of vIOMMU */
bool __read_mostly viommu_verbose;
boolean_runtime_param("viommu_verbose", viommu_verbose);

#ifndef NDEBUG
#define vvtd_info(fmt...) do {                    \
    if ( viommu_verbose )                         \
        gprintk(XENLOG_INFO, ## fmt);             \
} while(0)
/*
 * Use printk and '_G_' prefix because vvtd_debug() may be called
 * in the context of another domain's vCPU. Don't output 'current'
 * information to avoid confusion.
 */
#define vvtd_debug(fmt...) do {                   \
    if ( viommu_verbose && printk_ratelimit())    \
        printk(XENLOG_G_DEBUG fmt);               \
} while(0)
#else
#define vvtd_info(...) do {} while(0)
#define vvtd_debug(...) do {} while(0)
#endif

#define VVTD_REG_POS(vvtd, offset) &(vvtd->hw.regs[offset/sizeof(uint32_t)])

static inline void vvtd_set_reg(struct vvtd *vvtd, uint32_t reg, uint32_t value)
{
    *VVTD_REG_POS(vvtd, reg) = value;
}

static inline uint32_t vvtd_get_reg(const struct vvtd *vvtd, uint32_t reg)
{
    return *VVTD_REG_POS(vvtd, reg);
}

static inline void vvtd_set_reg_quad(struct vvtd *vvtd, uint32_t reg,
                                     uint64_t value)
{
    *(uint64_t*)VVTD_REG_POS(vvtd, reg) = value;
}

static inline uint64_t vvtd_get_reg_quad(const struct vvtd *vvtd, uint32_t reg)
{
    return *(uint64_t*)VVTD_REG_POS(vvtd, reg);
}

static void vvtd_reset(struct vvtd *vvtd)
{
    uint64_t cap = cap_set_num_fault_regs(VVTD_FRCD_NUM)
                   | cap_set_fault_reg_offset(VVTD_FRCD_START)
                   | cap_set_mgaw(39) /* maximum guest address width */
                   | cap_set_sagaw(2) /* support 3-level page_table */
                   | cap_set_ndoms(6); /* support 64K domains */
    uint64_t ecap = DMA_ECAP_QUEUED_INVAL | DMA_ECAP_INTR_REMAP | DMA_ECAP_EIM |
                    ecap_set_mhmv(0xf);

    vvtd_set_reg(vvtd, DMAR_VER_REG, 0x10UL);
    vvtd_set_reg_quad(vvtd, DMAR_CAP_REG, cap);
    vvtd_set_reg_quad(vvtd, DMAR_ECAP_REG, ecap);
    vvtd_set_reg(vvtd, DMAR_FECTL_REG, 0x80000000UL);
    vvtd_set_reg(vvtd, DMAR_IECTL_REG, 0x80000000UL);
}

static int vvtd_create(struct domain *d, struct viommu *viommu)
{
    struct vvtd *vvtd;

    if ( !is_hvm_domain(d) || (viommu->base_address & (PAGE_SIZE - 1)) ||
         (~VVTD_MAX_CAPS & viommu->caps) )
        return -EINVAL;

    vvtd = xzalloc_bytes(sizeof(struct vvtd));
    if ( !vvtd )
        return ENOMEM;

    vvtd_reset(vvtd);
    vvtd->base_addr = viommu->base_address;
    vvtd->domain = d;

    viommu->priv = vvtd;

    return 0;
}

static int vvtd_destroy(struct viommu *viommu)
{
    struct vvtd *vvtd = viommu->priv;

    if ( vvtd )
        xfree(vvtd);

    return 0;
}

static const struct viommu_ops vvtd_hvm_vmx_ops = {
    .create = vvtd_create,
    .destroy = vvtd_destroy,
};

REGISTER_VIOMMU(vvtd_hvm_vmx_ops);
