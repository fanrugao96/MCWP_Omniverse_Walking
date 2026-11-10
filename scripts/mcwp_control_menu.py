"""Reopen the existing MCWP panels without reinitializing any simulation state."""
from functools import partial

import omni.kit.actions.core
import omni.kit.menu.utils


class ControlWindowMenu:
    MENU_NAME = 'MCWP'
    ACTION_OWNER = 'mcwp.control_windows'

    def __init__(self, runtime):
        # Retain the original windows. A title-bar close only hides these panels.
        self.windows = {
            'lift': runtime.lift.window,
            'worker': runtime.worker.window,
            'simulation': runtime.window,
        }
        self.registry = omni.kit.actions.core.get_action_registry()
        self.items = []
        entries = [
            ('lift', 'MCWP Lift'),
            ('worker', 'Worker Remote'),
            ('simulation', 'MCWP Simulation Controls'),
            ('all', 'Show All Controls'),
        ]
        for key, label in entries:
            action_id = 'show_' + key
            callback = self.show_all if key == 'all' else partial(self.show, key)
            self.registry.register_action(
                self.ACTION_OWNER, action_id, callback,
                display_name=label,
                description='Show existing control window; preserve simulation state.',
            )
            if key == 'all':
                self.items.append(omni.kit.menu.utils.MenuItemDescription(name=''))
            self.items.append(omni.kit.menu.utils.MenuItemDescription(
                name=label, onclick_action=(self.ACTION_OWNER, action_id)))
        omni.kit.menu.utils.add_menu_items(self.items, self.MENU_NAME)

    def show(self, key):
        window = self.windows[key]
        window.visible = True
        window.focus()

    def show_all(self):
        for key in self.windows:
            self.show(key)

    def destroy(self):
        if self.items:
            omni.kit.menu.utils.remove_menu_items(self.items, self.MENU_NAME)
            self.registry.deregister_all_actions_for_extension(self.ACTION_OWNER)
            self.items = []
