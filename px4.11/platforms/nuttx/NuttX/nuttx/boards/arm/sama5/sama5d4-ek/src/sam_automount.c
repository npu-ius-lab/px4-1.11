/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sam_automount.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_FS_AUTOMOUNTER_DEBUG) && !defined(CONFIG_DEBUG_FS)
#  define CONFIG_DEBUG_FS 1
#endif

#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/fs/automount.h>

#include "sama5d4-ek.h"

#ifdef HAVE_AUTOMOUNTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL (FAR void *)0
#endif

#ifndef OK
#  define OK 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the changeable state of the automounter */

struct sam_automount_state_s
{
  volatile automount_handler_t handler;    /* Upper half handler */
  FAR void *arg;                           /* Handler argument */
  bool enable;                             /* Fake interrupt enable */
  bool pending;                            /* Set if there an event while disabled */
};

/* This structure represents the static configuration of an automounter */

struct sam_automount_config_s
{
  /* This must be first thing in structure so that we can simply cast from
   * struct automount_lower_s to struct sam_automount_config_s
   */

  struct automount_lower_s lower;          /* Publicly visible part */
  uint8_t hsmci;                           /* HSMCI0_SLOTNO or HSMCI1_SLOTNO */
  FAR struct sam_automount_state_s *state; /* Changeable state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  sam_attach(FAR const struct automount_lower_s *lower,
                       automount_handler_t isr, FAR void *arg);
static void sam_enable(FAR const struct automount_lower_s *lower,
                       bool enable);
static bool sam_inserted(FAR const struct automount_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT
static struct sam_automount_state_s g_hsmci0state;
static const struct sam_automount_config_s g_hsmci0config =
{
  .lower        =
  {
    .fstype     = CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_FSTYPE,
    .blockdev   = CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_BLKDEV,
    .mountpoint = CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_MOUNTPOINT,
    .ddelay     = MSEC2TICK(CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_DDELAY),
    .udelay     = MSEC2TICK(CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_UDELAY),
    .attach     = sam_attach,
    .enable     = sam_enable,
    .inserted   = sam_inserted
  },
  .hsmci        = HSMCI0_SLOTNO,
  .state        = &g_hsmci0state
};
#endif

#ifdef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT
static struct sam_automount_state_s g_hsmci1state;
static const struct sam_automount_config_s g_hsmci1config =
{
  .lower        =
  {
    .fstype     = CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_FSTYPE,
    .blockdev   = CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_BLKDEV,
    .mountpoint = CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_MOUNTPOINT,
    .ddelay     = MSEC2TICK(CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_DDELAY),
    .udelay     = MSEC2TICK(CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_UDELAY),
    .attach     = sam_attach,
    .enable     = sam_enable,
    .inserted   = sam_inserted
  },
  .hsmci        = HSMCI1_SLOTNO,
  .state        = &g_hsmci1state
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_attach
 *
 * Description:
 *   Attach a new HSMCI event handler
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *   isr   - The new event handler to be attach
 *   arg   - Client data to be provided when the event handler is invoked.
 *
 *  Returned Value:
 *    Always returns OK
 *
 ****************************************************************************/

static int sam_attach(FAR const struct automount_lower_s *lower,
                      automount_handler_t isr, FAR void *arg)
{
  FAR const struct sam_automount_config_s *config;
  FAR struct sam_automount_state_s *state;

  /* Recover references to our structure */

  config = (FAR struct sam_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

  state = config->state;

  /* Save the new handler info (clearing the handler first to eliminate race
   * conditions).
   */

  state->handler = NULL;
  state->pending = false;
  state->arg     = arg;
  state->handler = isr;
  return OK;
}

/****************************************************************************
 * Name:  sam_enable
 *
 * Description:
 *   Enable card insertion/removal event detection
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *   enable - True: enable event detection; False: disable
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

static void sam_enable(FAR const struct automount_lower_s *lower, bool enable)
{
  FAR const struct sam_automount_config_s *config;
  FAR struct sam_automount_state_s *state;
  irqstate_t flags;

  /* Recover references to our structure */

  config = (FAR struct sam_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

  state = config->state;

  /* Save the fake enable setting */

  flags = enter_critical_section();
  state->enable = enable;

  /* Did an interrupt occur while interrupts were disabled? */

  if (enable && state->pending)
    {
      /* Yes.. perform the fake interrupt if the interrutp is attached */

      if (state->handler)
        {
          bool inserted = sam_cardinserted(config->hsmci);
          (void)state->handler(&config->lower, state->arg, inserted);
        }

      state->pending = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_inserted
 *
 * Description:
 *   Check if a card is inserted into the slot.
 *
 * Input Parameters:
 *   lower - An instance of the auto-mounter lower half state structure
 *
 *  Returned Value:
 *    True if the card is inserted; False otherwise
 *
 ****************************************************************************/

static bool sam_inserted(FAR const struct automount_lower_s *lower)
{
  FAR const struct sam_automount_config_s *config;

  config = (FAR struct sam_automount_config_s *)lower;
  DEBUGASSERT(config && config->state);

  return sam_cardinserted(config->hsmci);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enable and so configured HSMCI
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

void sam_automount_initialize(void)
{
  FAR void *handle;

  finfo("Initializing automounter(s)\n");

#ifdef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT
  /* Initialize the HSMCI0 auto-mounter */

  handle = automount_initialize(&g_hsmci0config.lower);
  if (!handle)
    {
      ferr("ERROR: Failed to initialize auto-mounter for HSMCI0\n");
    }
#endif

#ifdef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT
  /* Initialize the HSMCI1 auto-mounter */

  handle = automount_initialize(&g_hsmci1config.lower);
  if (!handle)
    {
      ferr("ERROR: Failed to initialize auto-mounter for HSMCI1\n");
    }
#endif
}

/****************************************************************************
 * Name:  sam_automount_event
 *
 * Description:
 *   The HSMCI card detection logic has detected an insertion or removal
 *   event.
 *   It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the HSMCI0 slot: HSMCI0_SLOTNO or HSMCI1_SLOTNO.
 *      There is a terminology problem here:  Each HSMCI supports two slots,
 *      slot A and slot B.
 *      Only slot A is used.  So this is not a really a slot, but an HSCMI
 *      peripheral number.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

void sam_automount_event(int slotno, bool inserted)
{
  FAR const struct sam_automount_config_s *config;
  FAR struct sam_automount_state_s *state;

#ifdef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT
  /* Is this a change in the HSMCI0 insertion state? */

  if (slotno == HSMCI0_SLOTNO)
    {
      /* Yes.. Select the HSMCI0 automounter */

      config = &g_hsmci0config;
      state  = &g_hsmci0state;
    }
  else
#endif

#ifdef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT
  /* Is this a change in the HSMCI1 insertion state? */

  if (slotno == HSMCI1_SLOTNO)
    {
      /* Yes.. Select the HSMCI1 automounter */

      config = &g_hsmci1config;
      state  = &g_hsmci1state;
    }
  else
#endif
    {
      ferr("ERROR: Unsupported HSCMI%d\n", slotno);
      return;
    }

  /* Is the auto-mounter interrupt attached? */

  if (state->handler)
    {
      /* Yes.. Have we been asked to hold off interrupts? */

      if (!state->enable)
        {
          /* Yes.. just remember that there is a pending interrupt. We will
           * deliver the interrupt when interrupts are "re-enabled."
           */

          state->pending = true;
        }
      else
        {
          /* No.. forward the event to the handler */

          (void)state->handler(&config->lower, state->arg, inserted);
        }
    }
}

#endif /* HAVE_AUTOMOUNTER */
