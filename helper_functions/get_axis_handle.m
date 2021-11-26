%% get_axis_handle
%
% *Description*: This will extract an axes handle from a valid figure (or
% axes) handle, else it will return an error
%% Function Call
%
% *Inputs:* 
%
% _current_figORaxes_ (double) an axis or a figure handle or something else
%
% *Outputs:* 
%
% _curr_axis_ (double) an axes handle for plotting
%
% _curr_figure_ (double) the handle for the figure holding this axis
%
function [curr_axis, curr_figure] = get_axis_handle(current_figORaxes)
    
    if nargin == 0
        uiwait(msgbox('You have not provided a figure so I will create one, this may cause unexpected results'));
        current_figORaxes = gcf;
    end
    
    % If this fails then it is likely the figure or axes has been closed so open a new one
    try strcmp(get(current_figORaxes,'type'),'axes');   
    catch ME_1
        display(ME_1);
        figure(current_figORaxes);
    end
    
    curr_axis=0;     %#ok<NASGU>
    if strcmp(get(current_figORaxes,'type'),'axes')
        curr_axis=current_figORaxes;
    elseif strcmp(get(current_figORaxes,'type'),'figure')
        curr_axis=[];
        %get all children from figure
        curr_children=get(current_figORaxes,'Children');       
        %find an axes in the figure
        for curr_child=1:length(curr_children)
            if strcmp(get(curr_children(curr_child),'type'),'axes')
                curr_axis=curr_children(curr_child);
            end
        end
        %if there are none then create one and use the handle to that one
        if isempty(curr_axis)
            curr_axis=gca(current_figORaxes);
        end
    else
        error('You passed an invalid handle which is neither figure nor axes handle');
    end
    
    if isempty(getappdata(curr_axis,'axis_setup'))
        hold(curr_axis,'on')
        axis(curr_axis,'equal'); 
        view(curr_axis,3);
        set(curr_axis,'DrawMode','fast');
        setappdata(curr_axis,'axis_setup',true)
        try set(get(curr_axis,'Parent'),'Renderer','opengl'); end %#ok<TRYNC>
    end     
    
    if isempty(getappdata(curr_axis,'camlight_state')) || strcmp(getappdata(curr_axis,'camlight_state'),'off')
        lightPosition = getappdata(curr_axis,'lightPosition');
        if isempty(lightPosition)            
            lightPosition = [1 1 1];            
            setappdata(curr_axis,'lightPosition',lightPosition);
        end
        light('Position',lightPosition,'Style','infinite','Parent',curr_axis);
        setappdata(curr_axis,'camlight_state','on');
    end       
    
    %set the current axis to the be the current inside the figure which owns it
    set(get(curr_axis,'Parent'),'CurrentAxes',curr_axis);
    drawnow();
    
    if nargout==2
        curr_figure = get(curr_axis,'Parent');
    end
                
end
