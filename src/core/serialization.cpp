// Plain-text scene format (also used for undo/redo snapshots).
//
//   SAPPHIRE 3
//   world gx gy gz air elasticity ambient setpoint heat_rate viscosity width height depth next_group material
//   obj shape x y z vx vy vz qw qx qy qz wx wy wz motor_speed max may maz radius hx hy hz
//       liquid_radius elasticity mass material r g b a flags temperature latent_energy
//       molar_ratio fuse group
//   spring a b length stiffness damping ax ay az bx by bz
//   link a b length ice_bond ax ay az bx by bz
//   pulley a b anchor_x anchor_y anchor_z total_length
//   piston a b min max rest charge_T gas_T stiffness damping previous_length
//   axle wheel frame ax ay az
//   cluster n qw qx qy qz then n × (index rx ry rz)
//   container left right back front bottom wall_top piston_min piston_max piston_index group previous_height
//   end
//
// Object references are indices into the obj list (in file order).

#include "world.h"

#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace {
enum Flag : unsigned {
    kStatic = 1u << 0, kFluid = 1u << 1, kGas = 1u << 2, kMotor = 1u << 3, kBouncy = 1u << 4,
    kHeater = 1u << 5, kCooler = 1u << 6, kSteam = 1u << 7, kIce = 1u << 8, kContainerPiston = 1u << 9,
    kFixture = 1u << 10, kInCluster = 1u << 11,
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
    if (o.is_fixture) f |= kFixture;
    if (o.in_cluster) f |= kInCluster;
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
    o.is_fixture = f & kFixture;
    o.in_cluster = f & kInCluster;
}

bool fail(std::string* error, int line, const std::string& what) {
    if (error) *error = "line " + std::to_string(line) + ": " + what;
    return false;
}

std::ostream& operator<<(std::ostream& out, const Vec3& v) { return out << v.x << ' ' << v.y << ' ' << v.z; }
std::istream& operator>>(std::istream& in, Vec3& v) { return in >> v.x >> v.y >> v.z; }
} // namespace

std::string World::serialize() const {
    std::ostringstream out;
    out << std::setprecision(10);
    out << "SAPPHIRE 3\n";
    out << "world " << gravity << ' ' << air_density_scale << ' ' << elasticity << ' '
        << ambient_temperature << ' ' << ambient_setpoint << ' ' << heat_transfer_rate << ' '
        << fluid_viscosity << ' ' << width << ' ' << height << ' ' << depth << ' ' << next_group_ << ' '
        << int(spawn_material) << '\n';

    std::unordered_map<const PhysicsObject*, int> index;
    index.reserve(objects.size());
    for (std::size_t i = 0; i < objects.size(); ++i) {
        const PhysicsObject& o = *objects[i];
        index[&o] = int(i);
        out << "obj " << int(o.shape_type) << ' ' << o.position << ' ' << o.velocity << ' '
            << o.orientation.w << ' ' << o.orientation.x << ' ' << o.orientation.y << ' ' << o.orientation.z << ' '
            << o.angular_velocity << ' ' << o.motor_speed << ' ' << o.motor_axis << ' '
            << o.radius << ' ' << o.half_extents << ' ' << o.liquid_radius << ' ' << o.elasticity << ' '
            << o.mass << ' ' << int(o.material) << ' '
            << o.color.r << ' ' << o.color.g << ' ' << o.color.b << ' ' << o.color.a << ' '
            << pack_flags(o) << ' ' << o.temperature << ' ' << o.latent_energy << ' ' << o.molar_ratio << ' '
            << o.fuse << ' ' << o.group << '\n';
    }
    for (const auto& s : springs)
        out << "spring " << index[s.obj_a] << ' ' << index[s.obj_b] << ' ' << s.length << ' '
            << s.stiffness << ' ' << s.damping << ' ' << s.anchor_a << ' ' << s.anchor_b << '\n';
    for (const auto& l : rigid_links)
        out << "link " << index[l.obj_a] << ' ' << index[l.obj_b] << ' ' << l.length << ' ' << int(l.ice_bond)
            << ' ' << l.anchor_a << ' ' << l.anchor_b << '\n';
    for (const auto& p : pulleys)
        out << "pulley " << index[p.obj_a] << ' ' << index[p.obj_b] << ' ' << p.anchor << ' ' << p.total_length << '\n';
    for (const auto& p : pistons)
        out << "piston " << index[p.obj_a] << ' ' << index[p.obj_b] << ' ' << p.min_len << ' ' << p.max_len << ' '
            << p.rest_length << ' ' << p.charge_temperature << ' ' << p.gas_temperature << ' ' << p.stiffness << ' '
            << p.damping << ' ' << p.previous_length << '\n';
    for (const auto& a : axles)
        out << "axle " << index[a.wheel] << ' ' << index[a.frame] << ' ' << a.local_axis << '\n';
    for (const auto& c : clusters) {
        out << "cluster " << c.nodes.size() << ' ' << c.rotation.w << ' ' << c.rotation.x << ' '
            << c.rotation.y << ' ' << c.rotation.z;
        for (std::size_t i = 0; i < c.nodes.size(); ++i) out << ' ' << index[c.nodes[i]] << ' ' << c.rest[i];
        out << '\n';
    }
    for (const auto& c : containers)
        out << "container " << c.left << ' ' << c.right << ' ' << c.back << ' ' << c.front << ' ' << c.bottom << ' '
            << c.wall_top << ' ' << c.piston_min_y << ' ' << c.piston_max_y << ' '
            << (c.piston ? index[c.piston] : -1) << ' ' << c.group << ' ' << c.previous_chamber_height << '\n';
    out << "end\n";
    return out.str();
}

bool World::deserialize(const std::string& text, std::string* error) {
    std::istringstream in(text);
    std::string line;
    int line_no = 0;

    if (!std::getline(in, line) || line.rfind("SAPPHIRE", 0) != 0)
        return fail(error, 1, "not a Sapphire scene file");
    if (line.rfind("SAPPHIRE 3", 0) != 0)
        return fail(error, 1, "this scene was saved by the 2D version of Sapphire and cannot be opened in 3D");
    ++line_no;

    // Parse into temporaries so a bad file leaves the current scene intact.
    World next;
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
            int material = 0;
            ls >> next.gravity >> next.air_density_scale >> next.elasticity
               >> next.ambient_temperature >> next.ambient_setpoint >> next.heat_transfer_rate
               >> next.fluid_viscosity >> next.width >> next.height >> next.depth >> next.next_group_ >> material;
            if (!ls) return fail(error, line_no, "bad world record");
            if (next.width < 50.0 || next.height < 50.0 || next.depth < 50.0)
                return fail(error, line_no, "room is too small");
            next.spawn_material = material_from_int(material);
        } else if (kind == "obj") {
            int shape, material; unsigned flags;
            auto o = std::make_unique<PhysicsObject>();
            Quat& q = o->orientation;
            ls >> shape >> o->position >> o->velocity >> q.w >> q.x >> q.y >> q.z
               >> o->angular_velocity >> o->motor_speed >> o->motor_axis
               >> o->radius >> o->half_extents >> o->liquid_radius >> o->elasticity
               >> o->mass >> material >> o->color.r >> o->color.g >> o->color.b >> o->color.a >> flags
               >> o->temperature >> o->latent_energy >> o->molar_ratio >> o->fuse >> o->group;
            if (!ls) return fail(error, line_no, "bad object");
            if (!(o->mass > 0.0) || !(o->radius > 0.0)) return fail(error, line_no, "object needs positive mass and radius");
            o->shape_type = shape == int(ShapeType::BOX) ? ShapeType::BOX : ShapeType::SPHERE;
            if (o->shape_type == ShapeType::BOX &&
                !(o->half_extents.x > 0.0 && o->half_extents.y > 0.0 && o->half_extents.z > 0.0))
                return fail(error, line_no, "box needs positive half extents");
            if (!is_valid(o->position) || !is_valid(o->velocity) || !is_valid(q))
                return fail(error, line_no, "object has a non-finite pose");
            o->material = material_from_int(material);
            unpack_flags(*o, flags);
            // Only repair a clearly broken quaternion: renormalising one that is
            // merely rounded to 10 digits would make save → load → save drift.
            const double norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
            if (std::abs(norm2 - 1.0) > 1e-6) q.normalize();
            o->previous_orientation = q;
            o->previous_position = o->position;
            o->pre_velocity = o->velocity;
            o->pre_angular_velocity = o->angular_velocity;
            o->update_inertia();
            next.objects.push_back(std::move(o));
        } else if (kind == "spring") {
            int a, b; double len, k, d; Vec3 aa, ab;
            ls >> a >> b >> len >> k >> d >> aa >> ab;
            if (!ls || !object(a) || !object(b)) return fail(error, line_no, "bad spring");
            next.springs.emplace_back(object(a), object(b), len, k, d, aa, ab);
        } else if (kind == "link") {
            int a, b, ice; double len; Vec3 aa, ab;
            ls >> a >> b >> len >> ice >> aa >> ab;
            if (!ls || !object(a) || !object(b)) return fail(error, line_no, "bad link");
            next.rigid_links.emplace_back(object(a), object(b), len, ice != 0, aa, ab);
        } else if (kind == "pulley") {
            int a, b; Vec3 anchor; double total;
            ls >> a >> b >> anchor >> total;
            if (!ls || !object(a) || !object(b)) return fail(error, line_no, "bad pulley");
            next.pulleys.emplace_back(object(a), object(b), anchor, total);
        } else if (kind == "piston") {
            int a, b; double mn, mx, rest, charge, gas, k, d, prev;
            ls >> a >> b >> mn >> mx >> rest >> charge >> gas >> k >> d >> prev;
            if (!ls || !object(a) || !object(b)) return fail(error, line_no, "bad piston");
            PistonLink p(object(a), object(b), mn, mx, rest, charge, k, d);
            p.gas_temperature = gas;
            p.previous_length = prev;
            next.pistons.push_back(p);
        } else if (kind == "axle") {
            int wheel, frame; Vec3 axis;
            ls >> wheel >> frame >> axis;
            if (!ls || !object(wheel) || !object(frame)) return fail(error, line_no, "bad axle");
            next.axles.emplace_back(object(wheel), object(frame), axis);
        } else if (kind == "cluster") {
            std::size_t count = 0;
            RigidCluster c;
            ls >> count >> c.rotation.w >> c.rotation.x >> c.rotation.y >> c.rotation.z;
            if (!ls || count < 2 || count > 100000) return fail(error, line_no, "bad cluster");
            for (std::size_t i = 0; i < count; ++i) {
                int node; Vec3 rest;
                ls >> node >> rest;
                if (!ls || !object(node)) return fail(error, line_no, "bad cluster");
                c.nodes.push_back(object(node));
                c.rest.push_back(rest);
            }
            next.clusters.push_back(std::move(c));
        } else if (kind == "container") {
            double l, r, bk, fr, bottom, wall_top, pmin, pmax, prev; int piston, group;
            ls >> l >> r >> bk >> fr >> bottom >> wall_top >> pmin >> pmax >> piston >> group >> prev;
            if (!ls || !object(piston)) return fail(error, line_no, "bad container");
            GasContainer c(l, r, bk, fr, bottom, wall_top, pmin, pmax);
            c.piston = object(piston);
            c.group = group;
            c.previous_chamber_height = prev;
            next.containers.push_back(c);
        } else if (kind == "end") {
            ended = true;
            break;
        } else {
            return fail(error, line_no, "unknown record '" + kind + "'");
        }
    }
    if (!ended) return fail(error, line_no, "file is truncated");

    if (dragged_) dragged_->is_dragged = false;
    clear();
    objects = std::move(next.objects);
    springs = std::move(next.springs);
    rigid_links = std::move(next.rigid_links);
    pulleys = std::move(next.pulleys);
    pistons = std::move(next.pistons);
    axles = std::move(next.axles);
    clusters = std::move(next.clusters);
    containers = std::move(next.containers);
    gravity = next.gravity;
    air_density_scale = next.air_density_scale;
    elasticity = next.elasticity;
    ambient_temperature = next.ambient_temperature;
    ambient_setpoint = next.ambient_setpoint;
    heat_transfer_rate = next.heat_transfer_rate;
    fluid_viscosity = next.fluid_viscosity;
    width = next.width;
    height = next.height;
    depth = next.depth;
    spawn_material = next.spawn_material;
    next_group_ = std::max(next.next_group_, 1);
    for (const auto& o : objects) next_group_ = std::max(next_group_, o->group + 1);
    return true;
}
