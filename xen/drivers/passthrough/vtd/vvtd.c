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
#include <asm/apic.h>
#include <asm/current.h>
#include <asm/event.h>
#include <asm/io_apic.h>
#include <asm/hvm/domain.h>
#include <asm/p2m.h>

#include "iommu.h"
#include "vtd.h"

/* Supported capabilities by vvtd */
#define VVTD_MAX_CAPS VIOMMU_CAP_IRQ_REMAPPING

#define VVTD_FRCD_NUM   1ULL
#define VVTD_FRCD_START (DMAR_IRTA_REG + 8)
#define VVTD_FRCD_END   (VVTD_FRCD_START + VVTD_FRCD_NUM * 16)
#define VVTD_MAX_OFFSET VVTD_FRCD_END

struct hvm_hw_vvtd {
    bool eim_enabled;
    bool intremap_enabled;
    uint32_t fault_index;

    /* Interrupt remapping table base gfn and the max of entries */
    uint16_t irt_max_entry;
    gfn_t irt;

    uint32_t regs[VVTD_MAX_OFFSET/sizeof(uint32_t)];
};

struct vvtd {
    /* Base address of remapping hardware register-set */
    uint64_t base_addr;
    /* Point back to the owner domain */
    struct domain *domain;
    /* # of in-flight interrupts */
    atomic_t inflight_intr;
    /*
     * This lock protects fault-event related registers (DMAR_FEXXX_REG).
     * It's used for draining in-flight fault events before responding
     * guest's programming to those registers.
     */
    spinlock_t fe_lock;

    struct hvm_hw_vvtd hw;
    void *irt_base;
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
static inline int vvtd_test_and_set_bit(struct vvtd *vvtd, uint32_t reg, int nr)
{
    return test_and_set_bit(nr, VVTD_REG_POS(vvtd, reg));
}

static inline int vvtd_test_and_clear_bit(struct vvtd *vvtd, uint32_t reg,
                                          int nr)
{
    return test_and_clear_bit(nr, VVTD_REG_POS(vvtd, reg));
}

static inline int vvtd_test_bit(struct vvtd *vvtd, uint32_t reg, int nr)
{
    return test_bit(nr, VVTD_REG_POS(vvtd, reg));
}

static inline void vvtd_set_bit(struct vvtd *vvtd, uint32_t reg, int nr)
{
    __set_bit(nr, VVTD_REG_POS(vvtd, reg));
}

static inline void vvtd_clear_bit(struct vvtd *vvtd, uint32_t reg, int nr)
{
    __clear_bit(nr, VVTD_REG_POS(vvtd, reg));
}

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

static void *domain_vvtd(const struct domain *d)
{
    if ( is_hvm_domain(d) && d->arch.hvm_domain.viommu )
        return d->arch.hvm_domain.viommu->priv;
    else
        return NULL;
}

static void *map_guest_pages(struct domain *d, uint64_t gfn, uint32_t nr)
{
    mfn_t *mfn = xmalloc_array(mfn_t, nr);
    void* ret;
    int i;

    if ( !mfn )
        return NULL;

    for ( i = 0; i < nr; i++)
    {
        struct page_info *p = get_page_from_gfn(d, gfn + i, NULL, P2M_ALLOC);

        if ( !p || !get_page_type(p, PGT_writable_page) )
        {
            if ( p )
                put_page(p);
            goto undo;
        }

        mfn[i] = _mfn(page_to_mfn(p));
    }

    ret = vmap(mfn, nr);
    if ( ret == NULL )
        goto undo;
    xfree(mfn);

    return ret;

 undo:
    for ( ; --i >= 0; )
        put_page_and_type(mfn_to_page(mfn_x(mfn[i])));
    xfree(mfn);
    gprintk(XENLOG_ERR, "Failed to map guest pages %lx nr %x\n", gfn, nr);

    return NULL;
}

static void unmap_guest_pages(void *va, uint32_t nr)
{
    unsigned long *mfn = xmalloc_array(unsigned long, nr);
    int i;
    void *va_copy = va;

    if ( !mfn )
    {
        printk("%s %d: No free memory\n", __FILE__, __LINE__);
        return;
    }

    for ( i = 0; i < nr; i++, va += PAGE_SIZE)
        mfn[i] = domain_page_map_to_mfn(va);

    vunmap(va_copy);

    for ( i = 0; i < nr; i++)
        put_page_and_type(mfn_to_page(mfn[i]));
}

static int vvtd_delivery(struct domain *d, uint8_t vector,
                         uint32_t dest, bool dest_mode,
                         uint8_t delivery_mode, uint8_t trig_mode)
{
    struct vlapic *target;
    struct vcpu *v;

    switch ( delivery_mode )
    {
    case dest_LowestPrio:
        target = vlapic_lowest_prio(d, NULL, 0, dest, dest_mode);
        if ( target != NULL )
        {
            vvtd_debug("d%d: dest=v%d dlm=%x vector=%d trig_mode=%d\n",
                       vlapic_domain(target)->domain_id,
                       vlapic_vcpu(target)->vcpu_id,
                       delivery_mode, vector, trig_mode);
            vlapic_set_irq(target, vector, trig_mode);
            break;
        }
        vvtd_debug("d%d: null round robin: vector=%02x\n",
                   d->domain_id, vector);
        break;

    case dest_Fixed:
        for_each_vcpu ( d, v )
            if ( vlapic_match_dest(vcpu_vlapic(v), NULL, 0, dest, dest_mode) )
            {
                vvtd_debug("d%d: dest=v%d dlm=%x vector=%d trig_mode=%d\n",
                           v->domain->domain_id, v->vcpu_id,
                           delivery_mode, vector, trig_mode);
                vlapic_set_irq(vcpu_vlapic(v), vector, trig_mode);
            }
        break;

    case dest_NMI:
        for_each_vcpu ( d, v )
            if ( vlapic_match_dest(vcpu_vlapic(v), NULL, 0, dest, dest_mode) &&
                 !test_and_set_bool(v->nmi_pending) )
                vcpu_kick(v);
        break;

    default:
        gdprintk(XENLOG_WARNING, "Unsupported VTD delivery mode %d\n",
                 delivery_mode);
        return -EINVAL;
    }

    return 0;
}

static void vvtd_generate_interrupt(const struct vvtd *vvtd, uint64_t addr,
                                    uint32_t data)
{
    bool dm = addr & MSI_ADDR_DESTMODE_MASK;
    uint32_t dest = MASK_EXTR(addr, MSI_ADDR_DEST_ID_MASK);
    uint8_t dlm = MASK_EXTR(data, MSI_DATA_DELIVERY_MODE_MASK);
    uint8_t tm = MASK_EXTR(data, MSI_DATA_TRIGGER_MASK);
    uint8_t vector = data & MSI_DATA_VECTOR_MASK;

    vvtd_debug("d%d: generating msi %lx %x\n", vvtd->domain->domain_id, addr,
               data);

    if ( vvtd->hw.eim_enabled )
        dest |= (addr >> 40) << 8;

    vvtd_delivery(vvtd->domain, vector, dest, dm, dlm, tm);
}

static void vvtd_notify_fault(const struct vvtd *vvtd)
{
    vvtd_generate_interrupt(vvtd, vvtd_get_reg_quad(vvtd, DMAR_FEADDR_REG),
                            vvtd_get_reg(vvtd, DMAR_FEDATA_REG));
}

/* Computing the IRTE index for a given interrupt request. When success, return
 * 0 and set index to reference the corresponding IRTE. Otherwise, return < 0,
 * i.e. -1 when the irq request isn't an remapping format.
 */
static int irq_remapping_request_index(
    const struct arch_irq_remapping_request *irq, uint32_t *index)
{
    switch ( irq->type )
    {
    case VIOMMU_REQUEST_IRQ_MSI:
    {
        struct msi_msg_remap_entry msi_msg =
        {
            .address_lo = { .val = irq->msg.msi.addr },
            .data = irq->msg.msi.data,
        };

        if ( !msi_msg.address_lo.format )
            return -1;

        *index = (msi_msg.address_lo.index_15 << 15) +
                msi_msg.address_lo.index_0_14;
        if ( msi_msg.address_lo.SHV )
            *index += (uint16_t)msi_msg.data;
        break;
    }

    case VIOMMU_REQUEST_IRQ_APIC:
    {
        struct IO_APIC_route_remap_entry remap_rte = { .val = irq->msg.rte };

        if ( !remap_rte.format )
            return -1;

        *index = (remap_rte.index_15 << 15) + remap_rte.index_0_14;
        break;
    }

    default:
        ASSERT_UNREACHABLE();
    }

    return 0;
}

static inline uint32_t irte_dest(struct vvtd *vvtd, uint32_t dest)
{
    /* In xAPIC mode, only 8-bits([15:8]) are valid */
    return vvtd->hw.eim_enabled ? dest
                                : MASK_EXTR(dest, IRTE_xAPIC_DEST_MASK);
}

static void vvtd_report_non_recoverable_fault(struct vvtd *vvtd, int reason)
{
    uint32_t fsts = vvtd_get_reg(vvtd, DMAR_FSTS_REG);

    vvtd_set_bit(vvtd, DMAR_FSTS_REG, reason);

    /*
     * Accoroding to VT-d spec "Non-Recoverable Fault Event" chapter, if
     * there are any previously reported interrupt conditions that are yet to
     * be sevices by software, the Fault Event interrrupt is not generated.
     */
    if ( fsts & DMA_FSTS_FAULTS )
        return;

    vvtd_set_bit(vvtd, DMAR_FECTL_REG, DMA_FECTL_IP_SHIFT);
    if ( !vvtd_test_bit(vvtd, DMAR_FECTL_REG, DMA_FECTL_IM_SHIFT) )
    {
        vvtd_notify_fault(vvtd);
        vvtd_clear_bit(vvtd, DMAR_FECTL_REG, DMA_FECTL_IP_SHIFT);
    }
}

static void vvtd_update_ppf(struct vvtd *vvtd)
{
    int i;
    uint64_t cap = vvtd_get_reg_quad(vvtd, DMAR_CAP_REG);
    unsigned int base = cap_fault_reg_offset(cap);

    for ( i = 0; i < cap_num_fault_regs(cap); i++ )
    {
        if ( vvtd_test_bit(vvtd, base + i * DMA_FRCD_LEN + DMA_FRCD3_OFFSET,
                           DMA_FRCD_F_SHIFT) )
        {
            vvtd_report_non_recoverable_fault(vvtd, DMA_FSTS_PPF_SHIFT);
            return;
        }
    }
    /*
     * No Primary Fault is in Fault Record Registers, thus clear PPF bit in
     * FSTS.
     */
    vvtd_clear_bit(vvtd, DMAR_FSTS_REG, DMA_FSTS_PPF_SHIFT);

    /* If no fault is in FSTS, clear pending bit in FECTL. */
    if ( !(vvtd_get_reg(vvtd, DMAR_FSTS_REG) & DMA_FSTS_FAULTS) )
        vvtd_clear_bit(vvtd, DMAR_FECTL_REG, DMA_FECTL_IP_SHIFT);
}

/*
 * Commit a fault to emulated Fault Record Registers.
 */
static void vvtd_commit_frcd(struct vvtd *vvtd, int idx,
                             const struct vtd_fault_record_register *frcd)
{
    unsigned int base = cap_fault_reg_offset(
                            vvtd_get_reg_quad(vvtd, DMAR_CAP_REG));

    vvtd_set_reg_quad(vvtd, base + idx * DMA_FRCD_LEN, frcd->bits.lo);
    vvtd_set_reg_quad(vvtd, base + idx * DMA_FRCD_LEN + 8, frcd->bits.hi);
    vvtd_update_ppf(vvtd);
}

/*
 * Allocate a FRCD for the caller. If success, return the FRI. Or, return -1
 * when failure.
 */
static int vvtd_alloc_frcd(struct vvtd *vvtd)
{
    int prev;
    uint64_t cap = vvtd_get_reg_quad(vvtd, DMAR_CAP_REG);
    unsigned int base = cap_fault_reg_offset(cap);

    /* Set the F bit to indicate the FRCD is in use. */
    if ( !vvtd_test_and_set_bit(vvtd,
                                base + vvtd->hw.fault_index * DMA_FRCD_LEN +
                                DMA_FRCD3_OFFSET, DMA_FRCD_F_SHIFT) )
    {
        prev = vvtd->hw.fault_index;
        vvtd->hw.fault_index = (prev + 1) % cap_num_fault_regs(cap);
        return vvtd->hw.fault_index;
    }
    return -ENOMEM;
}

static void vvtd_free_frcd(struct vvtd *vvtd, int i)
{
    unsigned int base = cap_fault_reg_offset(
                            vvtd_get_reg_quad(vvtd, DMAR_CAP_REG));

    vvtd_clear_bit(vvtd, base + i * DMA_FRCD_LEN + DMA_FRCD3_OFFSET,
                   DMA_FRCD_F_SHIFT);
}

static int vvtd_record_fault(struct vvtd *vvtd,
                             const struct arch_irq_remapping_request *request,
                             int reason)
{
    struct vtd_fault_record_register frcd;
    int fault_index;
    uint32_t irt_index;

    spin_lock(&vvtd->fe_lock);
    switch(reason)
    {
    case VTD_FR_IR_REQ_RSVD:
    case VTD_FR_IR_INDEX_OVER:
    case VTD_FR_IR_ENTRY_P:
    case VTD_FR_IR_ROOT_INVAL:
    case VTD_FR_IR_IRTE_RSVD:
    case VTD_FR_IR_REQ_COMPAT:
    case VTD_FR_IR_SID_ERR:
        if ( vvtd_test_bit(vvtd, DMAR_FSTS_REG, DMA_FSTS_PFO_SHIFT) )
            goto out;

        /* No available Fault Record means Fault overflowed */
        fault_index = vvtd_alloc_frcd(vvtd);
        if ( fault_index < 0 )
        {
            vvtd_report_non_recoverable_fault(vvtd, DMA_FSTS_PFO_SHIFT);
            goto out;
        }
        memset(&frcd, 0, sizeof(frcd));
        frcd.fields.fault_reason = reason;
        if ( irq_remapping_request_index(request, &irt_index) )
            goto out;
        frcd.fields.fault_info = irt_index;
        frcd.fields.source_id = request->source_id;
        frcd.fields.fault = 1;
        vvtd_commit_frcd(vvtd, fault_index, &frcd);
        break;

    default:
        vvtd_debug("d%d: can't handle vvtd fault (reason 0x%x)",
                   vvtd->domain->domain_id, reason);
        break;
    }

 out:
    spin_unlock(&vvtd->fe_lock);
    return X86EMUL_OKAY;
}

static int vvtd_write_frcd3(struct vvtd *vvtd, uint32_t val)
{
    /* Writing a 1 means clear fault */
    if ( val & DMA_FRCD_F )
    {
        vvtd_free_frcd(vvtd, 0);
        vvtd_update_ppf(vvtd);
    }
    return X86EMUL_OKAY;
}

static void vvtd_write_fectl(struct vvtd *vvtd, uint32_t val)
{
    /*
     * Only DMA_FECTL_IM bit is writable. Generate pending event when unmask.
     */
    if ( !(val & DMA_FECTL_IM) )
    {
        /* Clear IM */
        vvtd_clear_bit(vvtd, DMAR_FECTL_REG, DMA_FECTL_IM_SHIFT);
        if ( vvtd_test_and_clear_bit(vvtd, DMAR_FECTL_REG, DMA_FECTL_IP_SHIFT) )
            vvtd_notify_fault(vvtd);
    }
    else
        vvtd_set_bit(vvtd, DMAR_FECTL_REG, DMA_FECTL_IM_SHIFT);
}

static void vvtd_write_fsts(struct vvtd *vvtd, uint32_t val)
{
    int i, max_fault_index = DMA_FSTS_PRO_SHIFT;
    uint64_t bits_to_clear = val & DMA_FSTS_RW1CS;

    if ( bits_to_clear )
    {
        i = find_first_bit(&bits_to_clear, max_fault_index / 8 + 1);
        while ( i <= max_fault_index )
        {
            vvtd_clear_bit(vvtd, DMAR_FSTS_REG, i);
            i = find_next_bit(&bits_to_clear, max_fault_index / 8 + 1, i + 1);
        }
    }

    /*
     * Clear IP field when all status fields in the Fault Status Register
     * being clear.
     */
    if ( !((vvtd_get_reg(vvtd, DMAR_FSTS_REG) & DMA_FSTS_FAULTS)) )
        vvtd_clear_bit(vvtd, DMAR_FECTL_REG, DMA_FECTL_IP_SHIFT);
}

static void write_gcmd_ire(struct vvtd *vvtd, uint32_t val)
{
    bool set = val & DMA_GCMD_IRE;

    vvtd_info("%sable Interrupt Remapping\n", set ? "En" : "Dis");

    vvtd->hw.intremap_enabled = set;
    (set ? vvtd_set_bit : vvtd_clear_bit)
        (vvtd, DMAR_GSTS_REG, DMA_GSTS_IRES_SHIFT);
}

static void write_gcmd_qie(struct vvtd *vvtd, uint32_t val)
{
    bool set = val & DMA_GCMD_QIE;

    vvtd_info("%sable Queue Invalidation\n", set ? "En" : "Dis");

    if ( set )
        vvtd_set_reg_quad(vvtd, DMAR_IQH_REG, 0);

    (set ? vvtd_set_bit : vvtd_clear_bit)
        (vvtd, DMAR_GSTS_REG, DMA_GSTS_QIES_SHIFT);

}

static void write_gcmd_sirtp(struct vvtd *vvtd, uint32_t val)
{
    uint64_t irta = vvtd_get_reg_quad(vvtd, DMAR_IRTA_REG);

    if ( !(val & DMA_GCMD_SIRTP) )
        return;

    /*
     * Hardware clears this bit when software sets the SIRTPS field in
     * the Global Command register and sets it when hardware completes
     * the 'Set Interrupt Remap Table Pointer' operation.
     */
    vvtd_clear_bit(vvtd, DMAR_GSTS_REG, DMA_GSTS_SIRTPS_SHIFT);
    if ( vvtd->hw.intremap_enabled )
        vvtd_info("Update Interrupt Remapping Table when active\n");

    if ( gfn_x(vvtd->hw.irt) != PFN_DOWN(DMA_IRTA_ADDR(irta)) ||
         vvtd->hw.irt_max_entry != DMA_IRTA_SIZE(irta) )
    {
        if ( vvtd->irt_base )
        {
            unmap_guest_pages(vvtd->irt_base,
                              PFN_UP(vvtd->hw.irt_max_entry *
                                     sizeof(struct iremap_entry)));
            vvtd->irt_base = NULL;
        }
        vvtd->hw.irt = _gfn(PFN_DOWN(DMA_IRTA_ADDR(irta)));
        vvtd->hw.irt_max_entry = DMA_IRTA_SIZE(irta);
        vvtd->hw.eim_enabled = !!(irta & IRTA_EIME);
        vvtd_info("Update IR info (addr=%lx eim=%d size=%d)\n",
                  gfn_x(vvtd->hw.irt), vvtd->hw.eim_enabled,
                  vvtd->hw.irt_max_entry);

        vvtd->irt_base = map_guest_pages(vvtd->domain, gfn_x(vvtd->hw.irt),
                                         PFN_UP(vvtd->hw.irt_max_entry *
                                                sizeof(struct iremap_entry)));
    }
    vvtd_set_bit(vvtd, DMAR_GSTS_REG, DMA_GSTS_SIRTPS_SHIFT);
}

static void vvtd_write_gcmd(struct vvtd *vvtd, uint32_t val)
{
    uint32_t orig = vvtd_get_reg(vvtd, DMAR_GSTS_REG);
    uint32_t changed;

    orig = orig & DMA_GCMD_ONE_SHOT_MASK;   /* reset the one-shot bits */
    changed = orig ^ val;

    if ( !changed )
        return;

    if ( changed & (changed - 1) )
        vvtd_info("Write %x to GCMD (current %x), updating multiple fields",
                  val, orig);

    if ( changed & DMA_GCMD_SIRTP )
        write_gcmd_sirtp(vvtd, val);
    if ( changed & DMA_GCMD_IRE )
        write_gcmd_ire(vvtd, val);
    if ( changed & DMA_GCMD_QIE )
        write_gcmd_qie(vvtd, val);
    if ( changed & ~(DMA_GCMD_SIRTP | DMA_GCMD_IRE | DMA_GCMD_QIE) )
        vvtd_info("Only SIRTP, IRE, QIE in GCMD are handled");
}

static int vvtd_in_range(struct vcpu *v, unsigned long addr)
{
    struct vvtd *vvtd = domain_vvtd(v->domain);

    if ( vvtd )
        return (addr >= vvtd->base_addr) &&
               (addr < vvtd->base_addr + VVTD_MAX_OFFSET);
    return 0;
}

static int vvtd_read(struct vcpu *v, unsigned long addr,
                     unsigned int len, unsigned long *pval)
{
    struct vvtd *vvtd = domain_vvtd(v->domain);
    unsigned int offset = addr - vvtd->base_addr;

    vvtd_info("Read offset %x len %d\n", offset, len);

    if ( (len != 4 && len != 8) || (offset & (len - 1)) )
        return X86EMUL_OKAY;

    if ( len == 4 )
        *pval = vvtd_get_reg(vvtd, offset);
    else
        *pval = vvtd_get_reg_quad(vvtd, offset);

    return X86EMUL_OKAY;
}

static void vvtd_write_fault_regs(struct vvtd *vvtd, unsigned long val,
                                  unsigned int offset, unsigned int len)
{
    unsigned int fault_offset = cap_fault_reg_offset(
                                    vvtd_get_reg_quad(vvtd, DMAR_CAP_REG));

    spin_lock(&vvtd->fe_lock);
    for ( ; len ; len -= 4, offset += 4, val = val >> 32)
    {
        switch ( offset )
        {
        case DMAR_FSTS_REG:
            vvtd_write_fsts(vvtd, val);
            break;

        case DMAR_FECTL_REG:
            vvtd_write_fectl(vvtd, val);
            break;

        case DMAR_FEDATA_REG:
        case DMAR_FEADDR_REG:
        case DMAR_FEUADDR_REG:
            vvtd_set_reg(vvtd, offset, val);
            break;

        default:
            if ( offset == (fault_offset + DMA_FRCD3_OFFSET) )
                 vvtd_write_frcd3(vvtd, val);
            break;
        }
    }
    spin_unlock(&vvtd->fe_lock);
}

static int vvtd_write(struct vcpu *v, unsigned long addr,
                      unsigned int len, unsigned long val)
{
    struct vvtd *vvtd = domain_vvtd(v->domain);
    unsigned int offset = addr - vvtd->base_addr;
    unsigned int fault_offset = cap_fault_reg_offset(
                                    vvtd_get_reg_quad(vvtd, DMAR_CAP_REG));

    vvtd_info("Write offset %x len %d val %lx\n", offset, len, val);

    if ( (len != 4 && len != 8) || (offset & (len - 1)) )
        return X86EMUL_OKAY;

    switch ( offset )
    {
    case DMAR_GCMD_REG:
        vvtd_write_gcmd(vvtd, val);
        break;

    case DMAR_IRTA_REG:
        vvtd_set_reg(vvtd, offset, val);
        if ( len == 4 )
            break;
        val = val >> 32;
        offset += 4;
        /* Fall through */
    case DMAR_IRTUA_REG:
        vvtd_set_reg(vvtd, offset, val);
        break;

    case DMAR_FSTS_REG:
    case DMAR_FECTL_REG:
    case DMAR_FEDATA_REG:
    case DMAR_FEADDR_REG:
    case DMAR_FEUADDR_REG:
        vvtd_write_fault_regs(vvtd, val, offset, len);
        break;

    default:
        if ( (offset == (fault_offset + DMA_FRCD2_OFFSET)) ||
             (offset == (fault_offset + DMA_FRCD3_OFFSET)) )
            vvtd_write_fault_regs(vvtd, val, offset, len);
        break;
    }

    return X86EMUL_OKAY;
}

static const struct hvm_mmio_ops vvtd_mmio_ops = {
    .check = vvtd_in_range,
    .read = vvtd_read,
    .write = vvtd_write
};

static void vvtd_handle_fault(struct vvtd *vvtd,
                              const struct arch_irq_remapping_request *irq,
                              struct iremap_entry *irte,
                              unsigned int fault)
{
    switch ( fault )
    {
    case VTD_FR_IR_SID_ERR:
    case VTD_FR_IR_IRTE_RSVD:
    case VTD_FR_IR_ENTRY_P:
        if ( qinval_fault_disable(*irte) )
            break;
    /* fall through */
    case VTD_FR_IR_REQ_RSVD:
    case VTD_FR_IR_INDEX_OVER:
    case VTD_FR_IR_ROOT_INVAL:
        vvtd_record_fault(vvtd, irq, fault);
        break;

    default:
        vvtd_debug("d%d can't handle VT-d fault %x\n", vvtd->domain->domain_id,
                   fault);
    }
    return;
}

static bool vvtd_irq_request_sanity_check(const struct vvtd *vvtd,
                                   const struct arch_irq_remapping_request *irq)
{
    switch ( irq->type )
    {
    case VIOMMU_REQUEST_IRQ_APIC:
    {
        struct IO_APIC_route_remap_entry rte = { .val = irq->msg.rte };

        return !rte.reserved;
    }

    case VIOMMU_REQUEST_IRQ_MSI:
        return true;
    }

    ASSERT_UNREACHABLE();
    return false;
}

static int vvtd_get_entry(struct vvtd *vvtd,
                          const struct arch_irq_remapping_request *irq,
                          struct iremap_entry *dest)
{
    uint32_t entry;
    struct iremap_entry irte;
    int ret = irq_remapping_request_index(irq, &entry);

    ASSERT(!ret);

    vvtd_debug("d%d: interpret a request with index %x\n",
               vvtd->domain->domain_id, entry);

    if ( !vvtd_irq_request_sanity_check(vvtd, irq) )
        return VTD_FR_IR_REQ_RSVD;
    else if ( entry > vvtd->hw.irt_max_entry )
        return VTD_FR_IR_INDEX_OVER;
    else if ( !vvtd->irt_base )
        return VTD_FR_IR_ROOT_INVAL;

    irte = ((struct iremap_entry*)vvtd->irt_base)[entry];

    if ( !qinval_present(irte) )
        ret = VTD_FR_IR_ENTRY_P;
    else if ( (irte.remap.res_1 || irte.remap.res_2 || irte.remap.res_3 ||
               irte.remap.res_4) )
        ret = VTD_FR_IR_IRTE_RSVD;

    /* FIXME: We don't check against the source ID */

    dest->val = irte.val;

    return ret;
}

static int vvtd_handle_irq_request(const struct domain *d,
                                   const struct arch_irq_remapping_request *irq)
{
    struct iremap_entry irte;
    int ret;
    struct vvtd *vvtd = domain_vvtd(d);

    if ( !vvtd || !vvtd->hw.intremap_enabled )
        return -ENODEV;

    atomic_inc(&vvtd->inflight_intr);
    ret = vvtd_get_entry(vvtd, irq, &irte);
    if ( ret )
    {
        vvtd_handle_fault(vvtd, irq, &irte, ret);
        goto out;
    }

    ret = vvtd_delivery(vvtd->domain, irte.remap.vector,
                        irte_dest(vvtd, irte.remap.dst),
                        irte.remap.dm, irte.remap.dlm,
                        irte.remap.tm);

 out:
    atomic_dec(&vvtd->inflight_intr);
    return ret;
}

static int vvtd_get_irq_info(const struct domain *d,
                             const struct arch_irq_remapping_request *irq,
                             struct arch_irq_remapping_info *info)
{
    int ret;
    struct iremap_entry irte;
    struct vvtd *vvtd = domain_vvtd(d);

    if ( !vvtd )
        return -ENODEV;

    ret = vvtd_get_entry(vvtd, irq, &irte);
    /* not in an interrupt delivery, don't report faults to guest */
    if ( ret )
        return ret;

    info->vector = irte.remap.vector;
    info->dest = irte_dest(vvtd, irte.remap.dst);
    info->dest_mode = irte.remap.dm;
    info->delivery_mode = irte.remap.dlm;

    return 0;
}

/* check whether the interrupt request is remappable */
static bool vvtd_is_remapping(const struct domain *d,
                              const struct arch_irq_remapping_request *irq)
{
    uint32_t idx;

    return !irq_remapping_request_index(irq, &idx);
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
    register_mmio_handler(d, &vvtd_mmio_ops);
    spin_lock_init(&vvtd->fe_lock);

    viommu->priv = vvtd;

    return 0;
}

static int vvtd_destroy(struct viommu *viommu)
{
    struct vvtd *vvtd = viommu->priv;

    if ( vvtd )
    {
        if ( vvtd->irt_base )
        {
            unmap_guest_pages(vvtd->irt_base,
                              PFN_UP(vvtd->hw.irt_max_entry *
                                     sizeof(struct iremap_entry)));
            vvtd->irt_base = NULL;
        }
        xfree(vvtd);
    }

    return 0;
}

static const struct viommu_ops vvtd_hvm_vmx_ops = {
    .create = vvtd_create,
    .destroy = vvtd_destroy,
    .handle_irq_request = vvtd_handle_irq_request,
    .get_irq_info = vvtd_get_irq_info,
    .check_irq_remapping = vvtd_is_remapping,
};

REGISTER_VIOMMU(vvtd_hvm_vmx_ops);
