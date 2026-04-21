#!/usr/bin/env python3

from pathlib import Path

from foxglove.layouts import (
    BaseRendererPointCloudTopicSettings,
    BaseRendererPosesTopicSettings,
    BaseRendererRosMarkerTopicSettings,
    BaseRendererRosPolygonTopicSettings,
    Layout,
    MarkdownConfig,
    MarkdownPanel,
    PlotConfig,
    PlotPanel,
    PlotSeries,
    RawMessagesConfig,
    RawMessagesPanel,
    SplitContainer,
    SplitItem,
    StackContainer,
    StackItem,
    TabContainer,
    TabItem,
    ThreeDeeConfig,
    ThreeDeePanel,
    TopicGraphPanel,
)


def build_layout() -> Layout:
    return Layout(
        content=TabContainer(
            selected_tab_index=0,
            tabs=[
                TabItem(
                    title="Navigation",
                    content=SplitContainer(
                        direction="row",
                        items=[
                            SplitItem(
                                proportion=3,
                                content=ThreeDeePanel(
                                    title="3D Navigation",
                                    config=ThreeDeeConfig(
                                        fixed_frame="map",
                                        topics={
                                            "/registered_scan": BaseRendererPointCloudTopicSettings(
                                                visible=True,
                                                point_size=2,
                                                point_shape="circle",
                                                decay_time=5.0,
                                                color_mode="colormap",
                                                color_field="intensity",
                                                color_map="rainbow",
                                                explicit_alpha=0.12,
                                            ),
                                            "/terrain_map": BaseRendererPointCloudTopicSettings(
                                                visible=False,
                                                point_size=3,
                                                point_shape="circle",
                                                color_mode="colormap",
                                                color_field="intensity",
                                                color_map="rainbow",
                                                explicit_alpha=0.8,
                                            ),
                                            "/terrain_map_ext": BaseRendererPointCloudTopicSettings(
                                                visible=False,
                                                point_size=3,
                                                point_shape="circle",
                                                color_mode="colormap",
                                                color_field="intensity",
                                                color_map="rainbow",
                                                explicit_alpha=0.8,
                                            ),
                                            "/overall_map": BaseRendererPointCloudTopicSettings(
                                                visible=False,
                                                point_size=2,
                                                point_shape="circle",
                                                color_mode="flat",
                                                flat_color="#FFFFFF",
                                                explicit_alpha=0.10,
                                            ),
                                            "/free_paths": BaseRendererPointCloudTopicSettings(
                                                visible=False,
                                                point_size=2,
                                                point_shape="circle",
                                                color_mode="colormap",
                                                color_field="intensity",
                                                color_map="rainbow",
                                                explicit_alpha=1.0,
                                            ),
                                            "/trajectory": BaseRendererPointCloudTopicSettings(
                                                visible=False,
                                                point_size=5,
                                                point_shape="circle",
                                                color_mode="colormap",
                                                color_field="intensity",
                                                color_map="rainbow",
                                            ),
                                            "/path": BaseRendererPosesTopicSettings(
                                                visible=True,
                                                type="line",
                                                line_width=0.08,
                                                gradient=("#19FF00", "#19FF00"),
                                            ),
                                            "/viz_graph_topic": BaseRendererRosMarkerTopicSettings(
                                                visible=True,
                                            ),
                                            "/graph_decoder_viz": BaseRendererRosMarkerTopicSettings(
                                                visible=False,
                                            ),
                                            "/navigation_boundary": BaseRendererRosPolygonTopicSettings(
                                                visible=True,
                                                line_width=0.08,
                                                color="#00FF00",
                                            ),
                                        },
                                    ),
                                ),
                            ),
                            SplitItem(
                                proportion=2,
                                content=StackContainer(
                                    title="Status",
                                    panels=[
                                        StackItem(
                                            size=0.34,
                                            panel=RawMessagesPanel(
                                                title="State Estimation",
                                                config=RawMessagesConfig(
                                                    topic_path="/state_estimation",
                                                    default_expanded=False,
                                                    expansion="all",
                                                    font_size=12,
                                                ),
                                            ),
                                        ),
                                        StackItem(
                                            size=0.33,
                                            panel=RawMessagesPanel(
                                                title="Goal Point",
                                                config=RawMessagesConfig(
                                                    topic_path="/goal_point",
                                                    default_expanded=True,
                                                    expansion="all",
                                                    font_size=12,
                                                ),
                                            ),
                                        ),
                                        StackItem(
                                            size=0.33,
                                            panel=RawMessagesPanel(
                                                title="FSM",
                                                config=RawMessagesConfig(
                                                    topic_path="/d15020108/rl_controller/fsm",
                                                    default_expanded=True,
                                                    expansion="all",
                                                    font_size=12,
                                                ),
                                            ),
                                        ),
                                    ],
                                ),
                            ),
                        ],
                    ),
                ),
                TabItem(
                    title="Plots",
                    content=SplitContainer(
                        direction="row",
                        items=[
                            SplitItem(
                                proportion=2,
                                content=PlotPanel(
                                    title="Robot Pose",
                                    config=PlotConfig(
                                        paths=[
                                            PlotSeries(
                                                value="/state_estimation.pose.pose.position.x",
                                                label="x",
                                                color="#00E5FF",
                                            ),
                                            PlotSeries(
                                                value="/state_estimation.pose.pose.position.y",
                                                label="y",
                                                color="#FFB300",
                                            ),
                                            PlotSeries(
                                                value="/state_estimation.pose.pose.position.z",
                                                label="z",
                                                color="#FF4D6D",
                                            ),
                                        ],
                                        show_legend=True,
                                        legend_display="top",
                                        time_range="latest",
                                        time_window_mode="sliding",
                                        following_view_width=20,
                                        x_axis_val="timestamp",
                                        y_axis_label="position",
                                    ),
                                ),
                            ),
                            SplitItem(
                                proportion=2,
                                content=PlotPanel(
                                    title="Robot Velocity",
                                    config=PlotConfig(
                                        paths=[
                                            PlotSeries(
                                                value="/state_estimation.twist.twist.linear.x",
                                                label="vx",
                                                color="#19FF00",
                                            ),
                                            PlotSeries(
                                                value="/state_estimation.twist.twist.linear.y",
                                                label="vy",
                                                color="#8E44AD",
                                            ),
                                            PlotSeries(
                                                value="/state_estimation.twist.twist.angular.z",
                                                label="wz",
                                                color="#FF6F00",
                                            ),
                                        ],
                                        show_legend=True,
                                        legend_display="top",
                                        time_range="latest",
                                        time_window_mode="sliding",
                                        following_view_width=20,
                                        x_axis_val="timestamp",
                                        y_axis_label="velocity",
                                    ),
                                ),
                            ),
                        ],
                    ),
                ),
                TabItem(
                    title="Graph",
                    content=SplitContainer(
                        direction="row",
                        items=[
                            SplitItem(
                                proportion=2,
                                content=TopicGraphPanel(title="Topic Graph"),
                            ),
                            SplitItem(
                                proportion=1,
                                content=MarkdownPanel(
                                    title="Notes",
                                    config=MarkdownConfig(
                                        markdown=(
                                            "# D1H + ODIN Foxglove\n\n"
                                            "Use this layout with `foxglove_bridge`.\n\n"
                                            "## Primary topics\n"
                                            "- `/registered_scan`\n"
                                            "- `/path`\n"
                                            "- `/viz_graph_topic`\n"
                                            "- `/state_estimation`\n"
                                            "- `/goal_point`\n\n"
                                            "This layout intentionally avoids TF-heavy views so the scene stays cleaner.\n\n"
                                            "## Heavy topics\n"
                                            "These are disabled by default because remote Wi-Fi may drop packets:\n"
                                            "- `/terrain_map`\n"
                                            "- `/terrain_map_ext`\n"
                                            "- `/overall_map`\n"
                                            "- `/graph_decoder_viz`\n\n"
                                            "## Fixed frame\n"
                                            "`map`\n"
                                        )
                                    ),
                                ),
                            ),
                        ],
                    ),
                ),
            ],
        )
    )


def build_exploration_debug_layout() -> Layout:
    return Layout(
        content=TabContainer(
            selected_tab_index=0,
            tabs=[
                TabItem(
                    title="Exploration Debug",
                    content=SplitContainer(
                        direction="row",
                        items=[
                            SplitItem(
                                proportion=3,
                                content=ThreeDeePanel(
                                    title="3D Exploration",
                                    config=ThreeDeeConfig(
                                        fixed_frame="map",
                                        topics={
                                            "/registered_scan": BaseRendererPointCloudTopicSettings(
                                                visible=True,
                                                point_size=2,
                                                point_shape="circle",
                                                decay_time=5.0,
                                                color_mode="colormap",
                                                color_field="intensity",
                                                color_map="rainbow",
                                                explicit_alpha=0.15,
                                            ),
                                            "/navigation_boundary": BaseRendererRosPolygonTopicSettings(
                                                visible=True,
                                                line_width=0.10,
                                                color="#00FF66",
                                            ),
                                            "/path": BaseRendererPosesTopicSettings(
                                                visible=True,
                                                type="line",
                                                line_width=0.10,
                                                gradient=("#19FF00", "#19FF00"),
                                            ),
                                            "/local_path": BaseRendererPosesTopicSettings(
                                                visible=True,
                                                type="line",
                                                line_width=0.09,
                                                gradient=("#00D1FF", "#00D1FF"),
                                            ),
                                            "/global_path": BaseRendererPosesTopicSettings(
                                                visible=False,
                                                type="line",
                                                line_width=0.08,
                                                gradient=("#FFB300", "#FFB300"),
                                            ),
                                            "/exploration_path": BaseRendererPosesTopicSettings(
                                                visible=False,
                                                type="line",
                                                line_width=0.08,
                                                gradient=("#FF4D6D", "#FF4D6D"),
                                            ),
                                            "/viewpoint_vis_cloud": BaseRendererPointCloudTopicSettings(
                                                visible=True,
                                                point_size=8,
                                                point_shape="circle",
                                                color_mode="flat",
                                                flat_color="#00E5FF",
                                                explicit_alpha=1.0,
                                            ),
                                            "/selected_viewpoint_vis_cloud": BaseRendererPointCloudTopicSettings(
                                                visible=True,
                                                point_size=14,
                                                point_shape="circle",
                                                color_mode="flat",
                                                flat_color="#FFD400",
                                                explicit_alpha=1.0,
                                            ),
                                            "/lookahead_point_cloud": BaseRendererPointCloudTopicSettings(
                                                visible=True,
                                                point_size=18,
                                                point_shape="circle",
                                                color_mode="flat",
                                                flat_color="#FF3B30",
                                                explicit_alpha=1.0,
                                            ),
                                            "/uncovered_cloud": BaseRendererPointCloudTopicSettings(
                                                visible=True,
                                                point_size=4,
                                                point_shape="circle",
                                                color_mode="flat",
                                                flat_color="#FFFFFF",
                                                explicit_alpha=0.95,
                                            ),
                                            "/uncovered_frontier_cloud": BaseRendererPointCloudTopicSettings(
                                                visible=True,
                                                point_size=6,
                                                point_shape="circle",
                                                color_mode="flat",
                                                flat_color="#FF7A00",
                                                explicit_alpha=1.0,
                                            ),
                                            "/tare_visualizer/exploring_subspaces": BaseRendererRosMarkerTopicSettings(
                                                visible=True,
                                            ),
                                            "/tare_visualizer/local_planning_horizon": BaseRendererRosMarkerTopicSettings(
                                                visible=True,
                                            ),
                                            "/grid_world_marker": BaseRendererRosMarkerTopicSettings(
                                                visible=False,
                                            ),
                                            "/keypose_graph_edge_marker": BaseRendererRosMarkerTopicSettings(
                                                visible=False,
                                            ),
                                        },
                                    ),
                                ),
                            ),
                            SplitItem(
                                proportion=2,
                                content=StackContainer(
                                    title="Debug Panels",
                                    panels=[
                                        StackItem(
                                            size=0.34,
                                            panel=RawMessagesPanel(
                                                title="Waypoint",
                                                config=RawMessagesConfig(
                                                    topic_path="/way_point",
                                                    default_expanded=True,
                                                    expansion="all",
                                                    font_size=12,
                                                ),
                                            ),
                                        ),
                                        StackItem(
                                            size=0.33,
                                            panel=RawMessagesPanel(
                                                title="State Estimation",
                                                config=RawMessagesConfig(
                                                    topic_path="/state_estimation",
                                                    default_expanded=False,
                                                    expansion="all",
                                                    font_size=12,
                                                ),
                                            ),
                                        ),
                                        StackItem(
                                            size=0.33,
                                            panel=MarkdownPanel(
                                                title="Notes",
                                                config=MarkdownConfig(
                                                    markdown=(
                                                        "# Exploration Debug\n\n"
                                                        "This layout is tuned for sparse exploration topics in Foxglove.\n\n"
                                                        "## What to watch\n"
                                                        "- `/viewpoint_vis_cloud`: current viewpoint set\n"
                                                        "- `/selected_viewpoint_vis_cloud`: chosen viewpoint\n"
                                                        "- `/lookahead_point_cloud`: current tracking target\n"
                                                        "- `/uncovered_cloud`: unexplored area when available\n"
                                                        "- `/uncovered_frontier_cloud`: frontier points when available\n\n"
                                                        "## Important\n"
                                                        "- Some exploration topics are intermittent.\n"
                                                        "- `exploring_subspaces` may publish a DELETE marker when no active subspace exists.\n"
                                                        "- The reliable exploration point topics are `/viewpoint_vis_cloud`, `/selected_viewpoint_vis_cloud`, and `/lookahead_point_cloud`.\n"
                                                        "- Do not rely on `/tare_visualizer/viewpoints`, `/tare_visualizer/viewpoint_candidates`, or `/tare_visualizer/uncovered_surface_points` for the current real-robot Foxglove workflow.\n"
                                                        "- If a layer looks empty, first confirm it is actually subscribed in Foxglove.\n"
                                                    )
                                                ),
                                            ),
                                        ),
                                    ],
                                ),
                            ),
                        ],
                    ),
                ),
                TabItem(
                    title="Plots",
                    content=SplitContainer(
                        direction="row",
                        items=[
                            SplitItem(
                                proportion=1,
                                content=PlotPanel(
                                    title="Robot Pose",
                                    config=PlotConfig(
                                        paths=[
                                            PlotSeries(
                                                value="/state_estimation.pose.pose.position.x",
                                                label="x",
                                                color="#00E5FF",
                                            ),
                                            PlotSeries(
                                                value="/state_estimation.pose.pose.position.y",
                                                label="y",
                                                color="#FFB300",
                                            ),
                                        ],
                                        show_legend=True,
                                        legend_display="top",
                                        time_range="latest",
                                        time_window_mode="sliding",
                                        following_view_width=20,
                                        x_axis_val="timestamp",
                                        y_axis_label="position",
                                    ),
                                ),
                            ),
                            SplitItem(
                                proportion=1,
                                content=PlotPanel(
                                    title="Velocity",
                                    config=PlotConfig(
                                        paths=[
                                            PlotSeries(
                                                value="/state_estimation.twist.twist.linear.x",
                                                label="vx",
                                                color="#19FF00",
                                            ),
                                            PlotSeries(
                                                value="/state_estimation.twist.twist.angular.z",
                                                label="wz",
                                                color="#FF6F00",
                                            ),
                                        ],
                                        show_legend=True,
                                        legend_display="top",
                                        time_range="latest",
                                        time_window_mode="sliding",
                                        following_view_width=20,
                                        x_axis_val="timestamp",
                                        y_axis_label="velocity",
                                    ),
                                ),
                            ),
                        ],
                    ),
                ),
            ],
        )
    )


def main() -> None:
    out_dir = Path("/home/robot/cmu_planner/docs/foxglove")
    out_dir.mkdir(parents=True, exist_ok=True)

    nav_layout = build_layout()
    nav_file = out_dir / "d1h_odin_navigation_layout.json"
    nav_file.write_text(nav_layout.to_json() + "\n")
    print(nav_file)

    exploration_layout = build_exploration_debug_layout()
    exploration_file = out_dir / "d1h_exploration_debug_layout.json"
    exploration_file.write_text(exploration_layout.to_json() + "\n")
    print(exploration_file)


if __name__ == "__main__":
    main()
