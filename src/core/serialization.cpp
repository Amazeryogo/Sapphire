// Plain-text scene format (also used for undo/redo snapshots).
//
//   SAPPHIRE 2
//   world gx gy air elasticity ambient setpoint heat_rate viscosity width height next_group
//   obj shape x y vx vy angle omega motor_speed radius liquid_radius elasticity mass r g b a
//       flags temperature latent_energy molar_ratio fuse group
//   spring a b length stiffness damping
//   link a b length ice_bond
//   pulley a b anchor_x anchor_y total_length
//   piston a b min max rest charge_T gas_T stiffness damping previous_length
//   container left right bottom wall_top piston_min piston_max piston_index previous_height
//   end
//
// Object references are indices into the obj list (in file order).

#include "world.h"

#include <iomanip>
#include <unordered_map>
#include <limits>
#include <sstream>

namespace {
enum Flag : unsigned {
    kStatic = 1u << 0, kFluid = 1u << 1, kGas = 1u << 2, kMotor = 1u << 3, kBouncy = 1u << 4,
    kHeater = 1u << 5, kCooler = 1u << 6, kSteam = 1u << 7, kIce = 1u << 8, kContainerPiston = 1u << 9,
};

unsigned pack_flags(const PhysicsObject& o) {
    unsigned f = 0;
    if (o.is_static) f |= kStatic;
    if (o.is_fluid) f |= kFluid;
    if (o.is_gas) f |= kGas;
    if (o.is_motor) f |= kMotor;
    if (o.is_bouncy) f |= kBouncy;
    if (o.is_heater) f |= kHeater;
    if (o.is_cooler) f |= kCooler;
    if (o.is_steam) f |= kSteam;
    if (o.is_ice) f |= kIce;
    if (o.is_container_piston) f |= kContainerPiston;
    return f;
}

void unpack_flags(PhysicsObject& o, unsigned f) {
    o.is_static = f & kStatic;
    o.is_fluid = f & kFluid;
    o.is_gas = f & kGas;
    o.is_motor = f & kMotor;
    o.is_bouncy = f & kBouncy;
    o.is_heater = f & kHeater;
    o.is_cooler = f & kCooler;
    o.is_steam = f & kSteam;
    o.is_ice = f & kIce;
    o.is_container_piston = f & kContainerPiston;
}

bool fail(std::string* error, int line, const std::string& what) {
    if (error) *error = "line " + std::to_string(line) + ": " + what;
    return false;
}
} // namespace

std::string World::serialize() const {
    std::ostringstream out;
    out << std::setprecision(10);
    out << "SAPPHIRE 2\n";
    out << "world " << gravity.x << ' ' << gravity.y << ' ' << air_resistance << ' ' << elasticity << ' '
        << ambient_temperature << ' ' << ambient_setpoint << ' ' << heat_transfer_rate << ' '
        << fluid_viscosity << ' ' << width << ' ' << height << ' ' << next_group_ << '\n';

    std::unordered_map<const PhysicsObject*, int> index;
    index.reserve(objects.size());
    for (std::size_t i = 0; i < objects.size(); ++i) {
        const PhysicsObject& o = *objects[i];
        index[&o] = int(i);
        out << "obj " << int(o.shape_type) << ' ' << o.position.x << ' ' << o.position.y << ' '
            << o.velocity.x << ' ' << o.velocity.y << ' ' << o.angle << ' ' << o.angular_velocity << ' '
            << o.motor_speed << ' ' << o.radius << ' ' << o.liquid_radius << ' ' << o.elasticity << ' '
            << o.mass << ' ' << o.color.r << ' ' << o.color.g << ' ' << o.color.b << ' ' << o.color.a << ' '
            << pack_flags(o) << ' ' << o.temperature << ' ' << o.latent_energy << ' ' << o.molar_ratio << ' '
            << o.fuse << ' ' << o.group << '\n';
    }
    for (const auto& s : springs)
        out << "spring " << index[s.obj_a] << ' ' << index[s.obj_b] << ' ' << s.length << ' '
            << s.stiffness << ' ' << s.damping << '\n';
    for (const auto& l : rigid_links)
        out << "link " << index[l.obj_a] << ' ' << index[l.obj_b] << ' ' << l.length << ' ' << int(l.ice_bond) << '\n';
    for (const auto& p : pulleys)
        out << "pulley " << index[p.obj_a] << ' ' << index[p.obj_b] << ' ' << p.anchor.x << ' ' << p.anchor.y
            << ' ' << p.total_length << '\n';
    for (const auto& p : pistons)
        out << "piston " << index[p.obj_a] << ' ' << index[p.obj_b] << ' ' << p.min_len << ' ' << p.max_len << ' '
            << p.rest_length << ' ' << p.charge_temperature << ' ' << p.gas_temperature << ' ' << p.stiffness << ' '
            << p.damping << ' ' << p.previous_length << '\n';
    for (const auto& c : containers)
        out << "container " << c.left << ' ' << c.right << ' ' << c.bottom << ' ' << c.wall_top << ' '
            << c.piston_min_y << ' ' << c.piston_max_y << ' ' << (c.piston ? index[c.piston] : -1) << ' '
            << c.previous_chamber_height << '\n';
    out << "end\n";
    return out.str();
}

bool World::deserialize(const std::string& text, std::string* error) {
    std::istringstream in(text);
    std::string line;
    int line_no = 0;

    if (!std::getline(in, line) || line.rfind("SAPPHIRE 2", 0) != 0)
        return fail(error, 1, "not a Sapphire scene file");
    ++line_no;

    // Parse into temporaries so a bad file leaves the current scene intact.
    World next;
    next.width = width;
    next.height = height;
    bool ended = false;
    auto object = [&](int idx) -> PhysicsObject* {
        return (idx >= 0 && idx < int(next.objects.size())) ? next.objects[std::size_t(idx)].get() : nullptr;
    };

    while (std::getline(in, line)) {
        ++line_no;
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ls(line);
        std::string kind;
        ls >> kind;
        if (kind == "world") {
            ls >> next.gravity.x >> next.gravity.y >> next.air_resistance >> next.elasticity
               >> next.ambient_temperature >> next.ambient_setpoint >> next.heat_transfer_rate
               >> next.fluid_viscosity >> next.width >> next.height >> next.next_group_;
        } else if (kind == "obj") {
            int shape; unsigned flags;
            auto o = std::make_unique<PhysicsObject>(0.0, 0.0);
            ls >> shape >> o->position.x >> o->position.y >> o->velocity.x >> o->velocity.y >> o->angle
               >> o->angular_velocity >> o->motor_speed >> o->radius >> o->liquid_radius >> o->elasticity
               >> o->mass >> o->color.r >> o->color.g >> o->color.b >> o->color.a >> flags >> o->temperature
               >> o->latent_energy >> o->molar_ratio >> o->fuse >> o->group;
            if (!ls) return fail(error, line_no, "bad object");
            if (o->mass <= 0.0 || o->radius <= 0.0) return fail(error, line_no, "object needs positive mass and radius");
            o->shape_type = shape == int(ShapeType::SQUARE) ? ShapeType::SQUARE : ShapeType::CIRCLE;
            unpack_flags(*o, flags);
            o->previous_position = o->position;
            o->pre_velocity = o->velocity;
            next.objects.push_back(std::move(o));
            continue;
        } else if (kind == "spring") {
            int a, b; double len, k, d;
            ls >> a >> b >> len >> k >> d;
            if (!ls || !object(a) || !object(b)) return fail(error, line_no, "bad spring");
            next.springs.emplace_back(object(a), object(b), len, k, d);
            continue;
        } else if (kind == "link") {
            int a, b, ice; double len;
            ls >> a >> b >> len >> ice;
            if (!ls || !object(a) || !object(b)) return fail(error, line_no, "bad link");
            next.rigid_links.emplace_back(object(a), object(b), len, ice != 0);
            continue;
        } else if (kind == "pulley") {
            int a, b; double ax, ay, total;
            ls >> a >> b >> ax >> ay >> total;
            if (!ls || !object(a) || !object(b)) return fail(error, line_no, "bad pulley");
            next.pulleys.emplace_back(object(a), object(b), Vector(ax, ay), total);
            continue;
        } else if (kind == "piston") {
            int a, b; double mn, mx, rest, charge, gas, k, d, prev;
            ls >> a >> b >> mn >> mx >> rest >> charge >> gas >> k >> d >> prev;
            if (!ls || !object(a) || !object(b)) return fail(error, line_no, "bad piston");
            PistonLink p(object(a), object(b), mn, mx, rest, charge, k, d);
            p.gas_temperature = gas;
            p.previous_length = prev;
            next.pistons.push_back(p);
            continue;
        } else if (kind == "container") {
            double l, r, bottom, wall_top, pmin, pmax, prev; int piston;
            ls >> l >> r >> bottom >> wall_top >> pmin >> pmax >> piston >> prev;
            if (!ls || !object(piston)) return fail(error, line_no, "bad container");
            GasContainer c(l, r, bottom, wall_top, pmin, pmax);
            c.piston = object(piston);
            c.previous_chamber_height = prev;
            next.containers.push_back(c);
            continue;
        } else if (kind == "end") {
            ended = true;
            break;
        } else {
            return fail(error, line_no, "unknown record '" + kind + "'");
        }
        if (!ls) return fail(error, line_no, "bad " + kind + " record");
    }
    if (!ended) return fail(error, line_no, "file is truncated");

    // Keep the live canvas size; the scene is simply placed in it.
    const double keep_width = width, keep_height = height;
    if (dragged_) dragged_->is_dragged = false;
    objects = std::move(next.objects);
    springs = std::move(next.springs);
    rigid_links = std::move(next.rigid_links);
    pulleys = std::move(next.pulleys);
    pistons = std::move(next.pistons);
    containers = std::move(next.containers);
    gravity = next.gravity;
    air_resistance = next.air_resistance;
    elasticity = next.elasticity;
    ambient_temperature = next.ambient_temperature;
    ambient_setpoint = next.ambient_setpoint;
    heat_transfer_rate = next.heat_transfer_rate;
    fluid_viscosity = next.fluid_viscosity;
    width = keep_width;
    height = keep_height;
    next_group_ = std::max(next.next_group_, 1);
    for (const auto& o : objects) next_group_ = std::max(next_group_, o->group + 1);
    dragged_ = nullptr;
    contacts_.clear();
    return true;
}
