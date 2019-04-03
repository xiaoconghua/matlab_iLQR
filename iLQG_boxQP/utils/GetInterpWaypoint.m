function interp_waypoint = GetInterpWaypoint(arclen, refTraj)
%GetInterpWaypoint: Return a waypoint given a arclength and reference
%trajectory.
    N = refTraj.num_nodes;
    refArcLength = refTraj.arclength;
    
    if (arclen <= refArcLength(1))
        interp_waypoint = refTraj.waypoints{1};
        return
    elseif (arclen >= refArcLength(end))
        interp_waypoint = refTraj.waypoints{end};
        return
    else     
        [closest_arc, closest_indx] = closest_value(refArcLength, arclen);
        if(arclen > closest_arc)
            lower_index = closest_indx;
            upper_index = closest_indx + 1;
        else
            lower_index = closest_indx - 1;
            upper_index = closest_indx;
        end

        if lower_index < 1 || upper_index > N
            disp('Index being out of bound');
        end
        interp_ratio = (arclen - refArcLength(lower_index))/(refArcLength(upper_index) - refArcLength(lower_index));
        interp_arclen  = interpValue(interp_ratio, refTraj.arclength(lower_index), refTraj.arclength(upper_index));
        if (abs(interp_arclen - arclen) > 0.01) 
            error('Arclength sanity check failed!');
        end
        interp_curv  = interpValue(interp_ratio, refTraj.curvature(lower_index), refTraj.curvature(upper_index));
        interp_vel  = interpValue(interp_ratio, refTraj.velocity(lower_index), refTraj.velocity(upper_index));

        interp_position = interpValue(interp_ratio, refTraj.position(lower_index, :), refTraj.position(upper_index, :));

        interp_waypoint = waypoint();
        interp_waypoint.curvature = interp_curv;
        interp_waypoint.velocity = interp_vel;
        interp_waypoint.arclength = interp_arclen;
        interp_waypoint.position = interp_position;
    end
end

function ret_value = interpValue(ratio, lower, upper)
    ret_value = lower + ratio.*(upper - lower);
end
