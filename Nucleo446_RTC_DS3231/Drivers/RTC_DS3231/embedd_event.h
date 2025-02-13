/*!
 * \file embedd_event.h
 * \brief Provides an interface to the event manager
 *
 * Software License Agreement:
 * 
 * This code is generated by the Embedd platform and is free to use, distribute, 
 * and modify by anyone, provided that a reference to the Embedd platform is 
 * included in any distributed or derivative works.
 * 
 * The code is provided "as is" without warranty of any kind, either expressed or 
 * implied, including but not limited to the implied warranties of merchantability 
 * and fitness for a particular purpose.
 * 
 * By using this code, you agree to acknowledge the Embedd platform in your 
 * projects or documentation where this code is utilized.
 * 
 * © 2024 Embedd Limited. All Rights Reserved.
 */

#ifndef _SRC_EMBEDD_EVENT_H
#define _SRC_EMBEDD_EVENT_H

#include "embedd_event_types.h"

/*!
 *  \fn     embedd_event_manager_init
 *  \brief  Initializing of event manager
 */
EMBEDD_RESULT embedd_event_manager_init();

/*!
 *  \fn     embedd_event_manager_deinit
 *  \brief  De-Initializing of event manager
 */
EMBEDD_RESULT embedd_event_manager_deinit();

/*!
 *  \fn     embedd_event_manager_register_callback
 *  \brief  register callback @cb for @id in event manager data base
 *
 *  \param  id  ID of event
 *  \param  cb  callback
 */
EMBEDD_RESULT embedd_event_manager_register_callback( uint32_t id, embedd_callback_t cb );   

/*!
 *  \fn     embedd_event_manager_register_oneshot
 *  \brief  register oneshot callback @cb for @id in event manager data base
 *
 *  \param  id  ID of event
 *  \param  cb  callback
 */
EMBEDD_RESULT embedd_event_manager_register_oneshot( uint32_t id, embedd_callback_t cb );

/*!
 *  \fn     embedd_event_manager_unregister_callback
 *  \brief  unregister callback @cb for @id in event manager data base
 *
 *  \param  id  ID of event
 *  \param  cb  callback
 */
EMBEDD_RESULT embedd_event_manager_unregister_callback(uint32_t id, embedd_callback_t cb );

/*!
 *  \fn     embedd_event_manager_process_events_enable
 *  \brief  enable event manager to process events in queue
 */
EMBEDD_RESULT embedd_event_manager_process_events_enable();

/*!
 *  \fn     embedd_event_manager_process_events_dsiable
 *  \brief  disable event manager to process events in queue
 */
EMBEDD_RESULT embedd_event_manager_process_events_disable();

/*!
 *  \fn     embedd_event_manager_process
 *  \brief  process events in queue by event manager
 */
EMBEDD_RESULT embedd_event_manager_process();

/*!
 *  \fn     embedd_event_manager_trigger
 *  \brief  add event to queue by event manager
 *
 *  \param  id    ID of event
 *  \param  data  data poiner, pointer to @embedd_device_t or pointer to @fsm_t
 */
EMBEDD_RESULT embedd_event_manager_trigger(uint32_t event_id, void *data);

#endif //_SRC_EMBEDD_EVENT_H
