function build_lib_internals(libPath, inports, outports, mfBlockName)
    add_block('built-in/TriggerPort', [libPath '/Trigger'], ...
        'TriggerType', 'function-call', 'Position', [30 30 60 60]);
    nIn = numel(inports);
    for k = 1:nIn
        y = 90 + (k-1)*40;
        add_block('built-in/Inport', [libPath '/' inports{k}], ...
            'Position', [30 y 60 y+14], 'Port', num2str(k));
    end
    nOut = numel(outports);
    blkH = max(nIn, nOut)*30 + 60;
    add_block('simulink/User-Defined Functions/MATLAB Function', [libPath '/' mfBlockName], ...
        'Position', [200 60 400 60+blkH]);
    for k = 1:nOut
        y = 90 + (k-1)*40;
        add_block('built-in/Outport', [libPath '/' outports{k}], ...
            'Position', [500 y 530 y+14], 'Port', num2str(k));
    end
end
