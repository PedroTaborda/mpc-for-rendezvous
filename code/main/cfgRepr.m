function reprStr = cfgRepr(cfg)
%cfgRepr Unique representation of a cfg struct as a string
    cfg.simulation.verbosity = 0;
    reprStr = repr(cfg);

end

function str = repr(obj)
    fields = fieldnames(obj);
    str = '';

    for i = 1:length(fields)
        field = fields{i};
        value = obj.(field);
        if isstruct(value)
            value = repr(value);
        elseif isnumeric(value)
            value = num2str(value);
        elseif ischar(value)
            value = ['''' value ''''];
        elseif islogical(value)
            value = num2str(value);
        elseif isa(value,'function_handle')
            value = char(value);
        elseif iscell(value)
            matval = cell2mat(value);
            if ~isempty(matval)
                value = repr(matval);
            else
                value = '';
            end
        else
            error('cfgRepr:unknownType', 'Unknown type of cfg field value');
        end
        % remove spaces
        valueStripped = strrep(mat2str(value), ' ', '');
        valueStripped = strrep(valueStripped, "'", '');
        str = sprintf('%s%s', str, valueStripped);
    end
end