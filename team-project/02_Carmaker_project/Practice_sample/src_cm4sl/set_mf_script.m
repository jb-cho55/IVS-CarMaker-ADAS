function set_mf_script(blockPath, scriptText)
    % Set the Script of a MATLAB Function block
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    if isempty(chart)
        error('No EMChart found at: %s', blockPath);
    end
    chart(1).Script = scriptText;
end
