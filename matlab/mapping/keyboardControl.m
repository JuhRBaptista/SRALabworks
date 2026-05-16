function keyboardControl(~, event)

    global control;

    dv = 0.025;
    dw = 0.05;

    switch event.Key

        case 'uparrow'
            control.v = control.v + dv;

        case 'downarrow'
            control.v = control.v - dv;

        case 'leftarrow'
            control.w = control.w + dw;

        case 'rightarrow'
            control.w = control.w - dw;

        case 'space'
            control.v = 0;
            control.w = 0;

        case 'escape'
            control.v = 0;
            control.w = 0;
            control.exit = true;

    end
end