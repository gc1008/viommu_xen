/*
 * include/asm-x86/viommu.h
 *
 * Copyright (c) 2017 Intel Corporation.
 * Author: Lan Tianyu <tianyu.lan@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef __ARCH_X86_VIOMMU_H__
#define __ARCH_X86_VIOMMU_H__

/* IRQ request type */
enum viommu_irq_request_type {
    VIOMMU_REQUEST_IRQ_MSI = 0,
    VIOMMU_REQUEST_IRQ_APIC = 1
};

struct arch_irq_remapping_info
{
    uint8_t dest_mode:1;
    uint8_t delivery_mode:3;
    uint8_t  vector;
    uint32_t dest;
};

struct arch_irq_remapping_request
{
    union {
        /* MSI */
        struct {
            uint64_t addr;
            uint32_t data;
        } msi;
        /* Redirection Entry in IOAPIC */
        uint64_t rte;
    } msg;
    uint16_t source_id;
    enum viommu_irq_request_type type;
};

#endif /* __ARCH_X86_VIOMMU_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
