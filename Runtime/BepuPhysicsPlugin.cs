namespace Engine.Physics.Bepu;

/// <summary>
/// Registers the BepuPhysics v2 implementation of <see cref="IPhysicsWorld"/>, the per-frame
/// physics step in <see cref="Stage.PreUpdate"/>, the transform write-back in <see cref="Stage.PostUpdate"/>,
/// and disposes the simulation in <see cref="Stage.Cleanup"/>.
/// </summary>
/// <remarks>
/// Add explicitly via <c>app.AddPlugin(new BepuPhysicsPlugin())</c>; included in
/// <see cref="DefaultPlugins"/> so games get rigid-body physics out-of-the-box.
/// To override <see cref="PhysicsSettings"/>, insert the resource before adding this plugin:
/// <code>
/// app.World.InsertResource(new PhysicsSettings { Gravity = new Vector3(0, -20f, 0) });
/// app.AddPlugin(new BepuPhysicsPlugin());
/// </code>
/// </remarks>
/// <example>
/// <code>
/// [Behavior]
/// public partial struct Player
/// {
///     public PhysicsBody Body;
///
///     [OnStartup]
///     public static void Spawn(BehaviorContext ctx)
///     {
///         var e = ctx.Ecs.Spawn();
///         var body = ctx.Physics.CreateCapsule(new Vector3(0, 5, 0), 0.5f, 1f);
///         ctx.Ecs.Add(e, new Player { Body = body });
///     }
/// }
/// </code>
/// </example>
public sealed class BepuPhysicsPlugin : IPlugin
{
    private static readonly ILogger Logger = Log.Category("Engine.Physics.Bepu");

    /// <inheritdoc />
    public void Build(App app)
    {
        Logger.Info("BepuPhysicsPlugin: initialising BepuPhysics v2 backend...");
        var settings = app.World.GetOrInsertResource(() => new PhysicsSettings());
        var world = new BepuPhysicsWorld(settings);
        app.World.InsertResource<IPhysicsWorld>(world);
        app.World.InsertResource(world); // also register the concrete type so disposal hits it via World.Dispose

        app.AddSystem(Stage.PreUpdate, new SystemDescriptor(static w =>
            {
                var phys = w.Resource<IPhysicsWorld>();
                var time = w.Resource<Time>();
                phys.Step((float)time.DeltaSeconds);
            }, "Physics.Step")
            .Read<Time>()
            .Write<IPhysicsWorld>()
            .MainThreadOnly());

        app.AddSystem(Stage.PostUpdate, new SystemDescriptor(static w =>
            {
                var phys = w.Resource<IPhysicsWorld>();
                var ecs = w.Resource<EcsWorld>();
                phys.SyncTransforms(ecs);
            }, "Physics.SyncTransforms")
            .Read<IPhysicsWorld>()
            .Write<EcsWorld>());

        Logger.Info("BepuPhysicsPlugin: physics systems registered (PreUpdate=Step, PostUpdate=SyncTransforms).");
    }
}

