function simulation (subject,hand, group, session,block)
         practice(subject,session,hand,1);
for i=1:block
switch group
    case 1
        skittles(subject,session,hand,i);
    case 2
        if session == 1
          skittlescore(subject,session,hand,i);
        else
            skittles_obstacles_for_exploration(subject,session,hand,i);
        end
    case 3
        if session == 1
          skittles_obstacles_for_exploration(subject,session,hand,i);
        else
          skittlescore(subject,session,hand,i);
        end 
end
end
end