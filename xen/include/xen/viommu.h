/*
 * include/xen/viommu.h
 *
 * Copyright (c) 2017, Intel Corporation
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
#ifndef __XEN_VIOMMU_H__
#define __XEN_VIOMMU_H__

#ifdef CONFIG_VIOMMU

#include <asm/viommu.h>

struct viommu;

struct viommu_ops {
    uint8_t type;
    int (*create)(struct domain *d, struct viommu *viommu);
    int (*destroy)(struct viommu *viommu);
    int (*handle_irq_request)(const struct domain *d,
                              const struct arch_irq_remapping_request *request);
    int (*get_irq_info)(const struct domain *d,
                        const struct arch_irq_remapping_request *request,
                        struct arch_irq_remapping_info *info);
};

struct viommu {
    uint64_t base_address;
    uint64_t caps;
    const struct viommu_ops *ops;
    void *priv;
};

#define REGISTER_VIOMMU(x) static const struct viommu_ops *x##_entry \
  __used_section(".data.viommus") = &x;


int viommu_register_type(uint8_t type, struct viommu_ops *ops);
int viommu_destroy_domain(struct domain *d);
int viommu_domctl(struct domain *d, struct xen_domctl_viommu_op *op);
int viommu_handle_irq_request(const struct domain *d,
                              const struct arch_irq_remapping_request *request);
int viommu_get_irq_info(const struct domain *d,
                        const struct arch_irq_remapping_request *request,
                        struct arch_irq_remapping_info *irq_info);
#else
static inline int viommu_destroy_domain(struct domain *d)
{
    return -EINVAL;
}
static inline
int viommu_domctl(struct domain *d, struct xen_domctl_viommu_op *op)
{
    return false;
}
#endif

#endif /* __XEN_VIOMMU_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
