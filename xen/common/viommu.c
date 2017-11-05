/*
 * common/viommu.c
 *
 * Copyright (c) 2017 Intel Corporation
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
 */

#include <xen/sched.h>
#include <xen/spinlock.h>
#include <xen/types.h>
#include <xen/viommu.h>

extern const struct viommu_ops *__start_viommus_array[], *__end_viommus_array[];
#define NUM_VIOMMU_TYPE (__end_viommus_array - __start_viommus_array)
#define viommu_type_array __start_viommus_array

int viommu_destroy_domain(struct domain *d)
{
    struct viommu *viommu = d->arch.hvm_domain.viommu;
    int ret;

    if ( !viommu )
        return -ENODEV;

    ret = viommu->ops->destroy(viommu);
    if ( ret < 0 )
        return ret;

    xfree(viommu);
    d->arch.hvm_domain.viommu = NULL;

    return 0;
}

static const struct viommu_ops *viommu_get_ops(uint8_t type)
{
    int i;

    for ( i = 0; i < NUM_VIOMMU_TYPE; i++)
    {
        if ( viommu_type_array[i]->type == type )
            return viommu_type_array[i];
    }

    return NULL;
}

static int viommu_create(struct domain *d, uint8_t type,
                         uint64_t base_address, uint64_t caps,
                         uint32_t *viommu_id)
{
    struct viommu *viommu;
    const struct viommu_ops *viommu_ops = NULL;
    int rc;

    /* Only support one vIOMMU per domain. */
    if ( d->arch.hvm_domain.viommu )
        return -E2BIG;

    viommu_ops = viommu_get_ops(type);
    if ( !viommu_ops )
        return -EINVAL;

    ASSERT(viommu_ops->create);

    viommu = xzalloc(struct viommu);
    if ( !viommu )
        return -ENOMEM;

    viommu->base_address = base_address;
    viommu->caps = caps;
    viommu->ops = viommu_ops;

    rc = viommu_ops->create(d, viommu);
    if ( rc < 0 )
    {
        xfree(viommu);
        return rc;
    }

    d->arch.hvm_domain.viommu = viommu;

    /* Only support one vIOMMU per domain. */
    *viommu_id = 0;
    return 0;
}

int viommu_domctl(struct domain *d, struct xen_domctl_viommu_op *op)
{
    int rc;

    switch ( op->cmd )
    {
    case XEN_DOMCTL_viommu_create:
        rc = viommu_create(d, op->u.create.type, op->u.create.base_address,
                           op->u.create.capabilities, &op->u.create.id);
        break;
    default:
        return -ENOSYS;
    }

    return rc;
}

int viommu_handle_irq_request(const struct domain *d,
                              const struct arch_irq_remapping_request *request)
{
    struct viommu *viommu = d->arch.hvm_domain.viommu;

    if ( !viommu )
        return -ENODEV;

    ASSERT(viommu->ops);
    if ( !viommu->ops->handle_irq_request )
        return -EINVAL;

    return viommu->ops->handle_irq_request(d, request);
}

int viommu_get_irq_info(const struct domain *d,
                        const struct arch_irq_remapping_request *request,
                        struct arch_irq_remapping_info *irq_info)
{
    const struct viommu *viommu = d->arch.hvm_domain.viommu;

    if ( !viommu )
        return -EINVAL;

    ASSERT(viommu->ops);
    if ( !viommu->ops->get_irq_info )
        return -EINVAL;

    return viommu->ops->get_irq_info(d, request, irq_info);
}

bool viommu_check_irq_remapping(const struct domain *d,
                                const struct arch_irq_remapping_request *request)
{
    const struct viommu *viommu = d->arch.hvm_domain.viommu;

    if ( !viommu )
        return false;

    ASSERT(viommu->ops);
    if ( !viommu->ops->check_irq_remapping )
        return false;

    return viommu->ops->check_irq_remapping(d, request);
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
