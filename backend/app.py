import flask
import requests
import json
import math

from flask import request
from flask_cors import CORS, cross_origin

app = flask.Flask(__name__)
cors = CORS(app)
app.config['CORS_HEADER'] = 'Content-Type'

def find_reaction_forces(supports: dict, external_forces: list) -> dict:
    reaction_forces = {x: {'x': None, 'y': None} for x in supports}
    num_pins = len([x for x in supports if supports[x]['type'] == 'pin'])
    num_rollers = len([x for x in supports if supports[x]['type'] == 'roller'])
    num_supports = num_pins + num_rollers
    if num_supports <= 2:
        sum_forces_x = sum([x['f_x'] for x in external_forces])
        sum_forces_y = sum([x['f_y'] for x in external_forces])
        if num_pins < 2:
            if sum_forces_x == 0:
                for support in supports:
                    if supports[support]['type'] == 'pin':
                        reaction_forces[support]['x'] = 0
            else:
                reaction_forces[[x for x in supports if supports[x]['type'] == 'pin'][0]]['x'] = -1 * sum_forces_x
        else:
            return {'status': 'failure', 'data': 'Question type not currently supported. Please try again at a later date.'}

        moment_loc = sorted(supports.keys())[0]
        moment_loc_x = supports[moment_loc]['pos_x']
        moment_loc_y = supports[moment_loc]['pos_y']

        external_forces_x = [[x['f_x'], x['pos_x'], x['pos_y']] for x in external_forces if x['f_x'] != 0]
        external_forces_y = [[x['f_y'], x['pos_x'], x['pos_y']] for x in external_forces if x['f_y'] != 0]
        sum_moments = 0
        for force, x, y in external_forces_x:
            sum_moments += (moment_loc_y - y) * force

        for force, x, y in external_forces_y:
            sum_moments += (x - moment_loc_x) * force

        support = sorted(supports.keys())[1]
        support_x = supports[support]['pos_x']
        support_y = supports[support]['pos_y']
        support_moment = -1 * sum_moments

        if moment_loc_x != support_x:
            support_reaction = support_moment / (support_x - moment_loc_x)
            reaction_forces[support]['y'] = support_reaction

        if supports[support]['type'] == 'pin':
            if moment_loc_y != support_y:
                support_reaction = support_moment / (moment_loc_y - support_y)
                reaction_forces[support]['y'] = support_reaction

        support_2 = sorted(supports.keys())[0]
        support_reaction_2 = -1 * sum_forces_y - support_reaction
        reaction_forces[support_2]['y'] = support_reaction_2

        return {'status': 'success', 'data': reaction_forces}

    return {'status': 'failure', 'data': 'Question type not currently supported. Please try again at a later date.'}

def find_internal_forces(joints: dict, members: dict) -> dict:
    sum_forces_x = 0
    for joint in joints:
        sum_forces_x += joints[joint]['f_x']

    if sum_forces_x != 0:
        return {'status': 'failure', 'data': 'X forces not balanced.'}

    sum_forces_y = 0
    for joint in joints:
        sum_forces_y += joints[joint]['f_y']

    if sum_forces_y != 0:
        return {'status': 'failure', 'data': 'Y forces not balanced.'}

    internal_forces = {x: {'force': None, 'type': None} for x in members}
    iterations = 0
    while len([x for x in internal_forces if internal_forces[x]['force'] == None]) != 0:
        for joint in joints:
            connecting_members = {x: {'x_force': None, 'y_force': None, 'x_dist': None, 'y_dist': None, 'total_dist': None} for x in members if joint in x}
            for member in connecting_members:
                x_distance = joints[joint]['pos_x'] - joints[member.replace(joint, '')]['pos_x']
                y_distance = joints[joint]['pos_y'] - joints[member.replace(joint, '')]['pos_y']
                total_distance = math.sqrt(x_distance ** 2 + y_distance ** 2)
                force = internal_forces[member]['force']
                type = internal_forces[member]['type']
                x_force = None
                y_force = None
                if x_distance != 0 and y_distance != 0:
                    #Diagonal
                    if force != None:
                        x_force = (abs(x_distance) / total_distance) * force
                        y_force = (abs(y_distance) / total_distance) * force
                        if x_distance > 0:
                            if type == 'T':
                                x_force *= -1
                        else:
                            if type == 'C':
                                x_force *= -1

                        if y_distance > 0:
                            if type == 'T':
                                y_force *= -1

                        else:
                            if type == 'C':
                                y_force *= -1

                elif x_distance != 0:
                    #Horizontal
                    y_force = 0
                    if force != None:
                        x_force = force
                        if x_distance > 0:
                            if type == 'T':
                                x_force *= -1
                        else:
                            if type == 'C':
                                x_force *= -1

                elif y_distance != 0:
                    #Vertical
                    x_force = 0
                    if force != None:
                        y_force = force
                        if y_distance > 0:
                            if type == 'T':
                                y_force *= -1

                        else:
                            if type == 'C':
                                y_force *= -1

                connecting_members[member]['x_force'] = x_force
                connecting_members[member]['y_force'] = y_force
                connecting_members[member]['x_dist'] = x_distance
                connecting_members[member]['y_dist'] = y_distance
                connecting_members[member]['total_dist'] = total_distance

            sum_forces_x = 0
            sum_forces_x += joints[joint]['f_x']
            x_unknowns = []
            for member in connecting_members:
                if connecting_members[member]['x_force'] != None:
                    sum_forces_x += connecting_members[member]['x_force']
                else:
                    x_unknowns.append(member)

            if len(x_unknowns) == 1:
                member = x_unknowns[0]
                unknown_force = (connecting_members[member]['total_dist'] / abs(connecting_members[member]['x_dist'])) * (-1 * sum_forces_x)
                if math.copysign(1, unknown_force) == math.copysign(1, connecting_members[member]['x_dist']):
                    type = 'C'
                else:
                    type = 'T'

                internal_forces[member] = {'force': abs(unknown_force), 'type': type}

            sum_forces_y = 0
            sum_forces_y += joints[joint]['f_y']
            y_unknowns = []
            for member in connecting_members:
                if connecting_members[member]['y_force'] != None:
                    sum_forces_y += connecting_members[member]['y_force']
                else:
                    y_unknowns.append(member)

            if len(y_unknowns) == 1:
                member = y_unknowns[0]
                unknown_force = (connecting_members[member]['total_dist'] / abs(connecting_members[member]['y_dist'])) * (-1 * sum_forces_y)
                if math.copysign(1, unknown_force) == math.copysign(1, connecting_members[member]['y_dist']):
                    type = 'C'
                else:
                    type = 'T'

                internal_forces[member] = {'force': abs(unknown_force), 'type': type}

        iterations += 1
        if iterations >= 500:
            return {'status': 'failure', 'data': 'Question type not currently supported. Please try again at a later date.'}

    return {'status': 'success', 'data': internal_forces}

@app.route('/solve_truss', methods=['GET'])
def solve_truss() -> dict:
    r = json.loads(request.data)

    joints = r['joints']
    supports = r['supports']
    external_forces = r['external_forces']
    members = r['members']

    result = find_reaction_forces(supports, external_forces)
    if result['status'] == 'success':
        support_reactions = result['data']
        for joint in support_reactions:
            joints[joint]['f_y'] += support_reactions[joint]['y']
            if support_reactions[joint]['x'] != None:
                joints[joint]['f_x'] += support_reactions[joint]['x']

        for force in external_forces:
            x_loc = force['pos_x']
            y_loc = force['pos_y']
            x_mag = force['f_x']
            y_mag = force['f_y']
            for joint in joints:
                if joints[joint]['pos_x'] == x_loc and joints[joint]['pos_y'] == y_loc:
                    joints[joint]['f_x'] += x_mag
                    joints[joint]['f_y'] += y_mag

        result = find_internal_forces(joints, members)
        if result['status'] == 'success':
            internal_forces = result['data']
            result = {'status': 'success', 'data': {'Support Reactions': support_reactions, 'Internal Forces': internal_forces}}

    return result

app.run()
