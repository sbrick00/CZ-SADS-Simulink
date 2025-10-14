function out = simFromStruct(name,struct)
    % Push every field of S into the sim’s variable space and run sim
    in  = Simulink.SimulationInput(name);
    fn = fieldnames(struct);
    for i = 1:numel(fn)
        in = in.setVariable(fn{i}, struct.(fn{i}));
    end
    out = sim(in);
end