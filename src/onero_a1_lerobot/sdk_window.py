"""Main-thread MuJoCo renderer with press/release events for simulation jogging."""

from contextlib import nullcontext


class SdkSimulationWindow:
    def __init__(self, model, data, *, key_callback, focus_callback):
        import glfw
        import mujoco

        self._glfw, self._mj = glfw, mujoco
        self._model, self._data = model, data
        self._window = self._context = None
        self._texts = None
        # mujoco 3.12 dropped the MjvScene parameter from mjv_moveCamera; older
        # releases take it between the mouse deltas and the camera. Detect it
        # once here so a mouse drag never raises from inside a GLFW callback.
        self._move_camera_takes_scene = "MjvScene" in (mujoco.mjv_moveCamera.__doc__ or "")
        if not glfw.init():
            raise RuntimeError("Cannot initialize the simulation desktop window")
        try:
            self._window = glfw.create_window(1200, 800, "Onero SDK simulation", None, None)
            if not self._window:
                raise RuntimeError("Cannot create the simulation desktop window")
            glfw.make_context_current(self._window)
            glfw.swap_interval(0)  # The session controls timing, not display refresh.
            self.cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(self.cam)
            self._option = mujoco.MjvOption()
            self._scene = mujoco.MjvScene(model, maxgeom=10000)
            self.user_scn = mujoco.MjvScene(model, maxgeom=128)
            self._context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
            self._cursor = glfw.get_cursor_pos(self._window)
            # Consume PRESS and RELEASE. OS key repeat must not alter jog speed.
            glfw.set_key_callback(self._window, lambda w, key, scan, action, mods: key_callback(key, action))
            glfw.set_window_focus_callback(self._window, lambda w, focused: focus_callback(bool(focused)))
            glfw.set_cursor_pos_callback(self._window, self._mouse_move)
            glfw.set_scroll_callback(self._window, self._scroll)
        except BaseException:
            self.close()
            raise

    def lock(self):
        return nullcontext()  # All GLFW/GL calls run on the control/main thread.

    def poll_events(self):
        self._glfw.poll_events()

    def is_running(self):
        return self._window is not None and not self._glfw.window_should_close(self._window)

    def set_texts(self, texts):
        self._texts = texts

    def _move_camera(self, action, reldx, reldy):
        scene = (self._scene,) if self._move_camera_takes_scene else ()
        self._mj.mjv_moveCamera(self._model, action, reldx, reldy, *scene, self.cam)

    def _mouse_move(self, window, x, y):
        glfw, mj = self._glfw, self._mj
        dx, dy = x - self._cursor[0], y - self._cursor[1]
        self._cursor = x, y
        height = max(glfw.get_window_size(window)[1], 1)
        shift = any(glfw.get_key(window, key) == glfw.PRESS for key in (glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT))
        if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS:
            action = mj.mjtMouse.mjMOUSE_MOVE_H if shift else mj.mjtMouse.mjMOUSE_MOVE_V
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H if shift else mj.mjtMouse.mjMOUSE_ROTATE_V
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS:
            action = mj.mjtMouse.mjMOUSE_ZOOM
        else:
            return
        self._move_camera(action, dx / height, dy / height)

    def _scroll(self, window, xoffset, yoffset):
        self._move_camera(self._mj.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * yoffset)

    def sync(self):
        if not self.is_running():
            return
        glfw, mj = self._glfw, self._mj
        glfw.make_context_current(self._window)
        width, height = glfw.get_framebuffer_size(self._window)
        if width == 0 or height == 0:
            return
        viewport = mj.MjrRect(0, 0, width, height)
        mj.mjv_updateScene(
            self._model, self._data, self._option, None, self.cam,
            mj.mjtCatBit.mjCAT_ALL, self._scene,
        )
        for i in range(self.user_scn.ngeom):
            source = self.user_scn.geoms[i]
            dest = self._scene.geoms[self._scene.ngeom]
            mj.mjv_initGeom(dest, source.type, source.size, source.pos, source.mat.ravel(), source.rgba)
            dest.label = source.label
            self._scene.ngeom += 1
        mj.mjr_render(viewport, self._scene, self._context)
        if self._texts:
            mj.mjr_overlay(*self._texts[:2], viewport, *self._texts[2:], self._context)
        glfw.swap_buffers(self._window)

    def close(self):
        try:
            if self._window:
                self._glfw.make_context_current(self._window)
            if self._context:
                self._context.free()
                self._context = None
        finally:
            if self._window:
                self._glfw.destroy_window(self._window)
                self._window = None
            self._glfw.terminate()
