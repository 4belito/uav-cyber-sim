import numpy as np
from plan.actions.navegation import get_local_position,make_go_to
from helpers.change_coordinates import local2global,global2local
from vehicle_logic import VehicleLogic



class GCS:
    def __init__(self,positions:np.ndarray):
        self.poss=positions.astype(float) 
        self.n_uavs=len(positions)
        self.incomplete_missions=set(range(1,self.n_uavs+1))


    def update_missions(self,completed_missions:set):
        self.incomplete_missions-=completed_missions


    def get_nearest_neighbor(self,uav):
        target_position = local2global(uav.curr_pos,uav.home,pairwise=True)  # Get the position of the target ID
        
        min_distance = float('inf')
        nearest_id = None

        for i, pos in enumerate(self.poss):
            if i== uav.sys_id-1:
                continue  # Skip itself
            
            distance = np.linalg.norm(target_position - pos)  # Compute Euclidean distance
            
            if distance < min_distance:
                min_distance = distance
                nearest_id = i+1  # Update nearest ID
        
        return nearest_id,min_distance
    


    def ask_position(self,uav: VehicleLogic):
        #print(f'uav asked {uav.sys_id}')
        local_pos=get_local_position(uav.conn,blocking=True)
        if local_pos is not False:
            uav.set_current_position(local_pos)
            self.poss[uav.sys_id-1] = local2global(local_pos,uav.home,pairwise=True)

            

    def send_neighborg_position(self,uav:VehicleLogic,neighborg_id:int):
        wp=uav.plan.current.current.wp
        
        print(f' pos {local2global(uav.curr_pos,uav.home,pairwise=True) }')
        print(f' obj {self.poss[neighborg_id-1]}')
        print(f'goal wp {local2global(wp,uav.home,pairwise=True)}')
        loacl_obj_pos=global2local(self.poss[neighborg_id-1],uav.home,pairwise=True)
        next_wp = GCS.get_avoidance_wp(pos=uav.curr_pos,
                                    obj_pos=loacl_obj_pos,
                                    goal_pos=wp)
        next_step=make_go_to(wp=next_wp,wp_margin=0.5,verbose=2,cause_text='(avoidance)')
        next_step.bind_connection(uav.conn)
        uav.plan.current.add_now(next_step)





    def get_avoidance_wp(pos: np.ndarray,
                    obj_pos:np.ndarray,
                    goal_pos:np.ndarray,
                    distance:float = 1,
                    direction: str = 'left'):
        """
        Sends a velocity command in body frame, orthogonal to the direction of wp.
        `direction` can be 'left' or 'right' (relative to wp direction).
        """

        # Normalize wp direction (ignore Z)
        obj_dir = (obj_pos -pos)[:2]
        goal_dir = (goal_pos -pos)[:2]
        if np.dot(obj_dir,goal_dir)<0:
            return goal_pos
        obj_dir = obj_dir / np.linalg.norm(obj_dir)*distance
        # Get orthogonal direction
        if direction == 'left':
            ortho = np.array([-obj_dir[1], obj_dir[0],0])
        elif direction == 'right':
            ortho = np.array([obj_dir[1], -obj_dir[0],0])
        else:
            raise ValueError("Direction must be 'left' or 'right'")

        # Scale to desired speed
        return pos+ortho


