import pyglet

window = pyglet.window.Window()

@window.event
def on_draw():
    print('on_draw')
    window.clear()

@window.event
def on_mouse_press(x, y, button, modifier):
    print("test")
    if modifier == pyglet.window.key.MOD_SHIFT:
        print("mod shift working")

@window.event
def on_key_press(symbol, modifiers):
    print('on_key_press', symbol, modifiers)

pyglet.app.run()