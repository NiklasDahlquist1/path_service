
#include "bt_generation/bt_generation.hpp"
// #include "bt_generation/file_management_functions.hpp"

namespace bt_generation
{


std::vector<std::string> split_string (std::string s, std::string delimiter) 
{
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}


std::weak_ptr<Node_bt> Node_bt::get_parent()
{
    return this->parent;
}

std::vector<std::shared_ptr<Node_bt>> Node_bt::get_children()
{
    return this->children;
}

std::string Node_bt::get_node_name()
{
    return this->node_name;
}

behavior_definition Node_bt::get_behavior_definition()
{
    return this->definition;
}

// std::optional<xml_attribute> get_xml_attribute_by_name(const std::string& name)
// {
//     for(const xml_attribute& attr : this->attributes)
//     {
//         if(attr.name == name)
//         {
//             return attr;
//         }
//     }
//     return std::nullopt;
// }



Node_bt::Node_bt(std::string node_name, std::string xml_type, std::shared_ptr<Node_bt> parent)
{            
    // TODO: separate node_name and definition name properly...
    this->node_name = node_name;
    this->definition.name = node_name;
    this->definition.xml_type = xml_type;
    this->parent = parent;
}
Node_bt::Node_bt(behavior_definition definition, std::shared_ptr<Node_bt> parent)
{
    this->node_name = definition.name;
    this->definition = definition;
    this->parent = parent;
}
Node_bt::Node_bt(std::shared_ptr<Node_bt> parent)
{
    this->node_name = "DEFAULT";
    this->parent = parent;
}

Node_bt::~Node_bt()
{
    // std::cout << "Destroyed: " << this->node_name << std::endl;
}


void Node_bt::set_parent(std::shared_ptr<Node_bt> parent)
{
    this->parent = parent;
}
void Node_bt::set_xml_type(const std::string& xml_type)
{
    this->definition.xml_type = xml_type;
}

void Node_bt::add_child(std::shared_ptr<Node_bt> child)
{
    child.get()->set_parent(this->shared_from_this());
    this->children.push_back(child); // Is this good enough?
}

void Node_bt::add_xml_attribute(std::string name, std::string value)
{
    xml_attribute attr;
    attr.name = name;
    attr.value = value;
    this->definition.attributes.push_back(attr);

    return;
}

void Node_bt::replace_child(const std::shared_ptr<Node_bt>& old_child, const std::shared_ptr<Node_bt>& new_child) 
{
    // bool replaced = false;
    int i = 0;
    // replace the pointer stored in this->children that matches the 'old_child'
    for(const std::shared_ptr<Node_bt>& c : this->children)
    {
        if(c == old_child)
        {

            // std::cout << "replacing node, found old_child: " << c.get()->get_node_name() << ". For root: " << this->node_name << "\n";
            new_child.get()->set_parent(this->shared_from_this());
            this->children[i] = std::move(new_child);
            // std::cout << "node_replaced\n";
            // replaced = true;
            break;
        }
        i++;
    }

    // if(replaced == false)
    // {

    //     std::cout << "Nothing was replaced, maybe child node does not exist? old child:  " << old_child.get()->get_node_name() << ". For root: " << this->node_name <<  "\n";
    // }
}

// repalce node in graph. Currently it will remove the same node multiple times if it exists (exhaustive search of all children)
void Node_bt::replace_node_in_sub_graph(const std::shared_ptr<Node_bt>& old_child, const std::shared_ptr<Node_bt>& new_child) 
{
    int i = 0;
    // replace the pointer stored in this->children that matches the 'old_child'
    for(const std::shared_ptr<Node_bt>& c : this->children)
    {
        if(c == old_child)
        {
            // std::cout << "replacing node, found old_child: " << c.get()->get_node_name() << ". For root: " << this->node_name << "\n";
            new_child.get()->set_parent(this->shared_from_this());
            this->children[i] = std::move(new_child);
            return;
        }
        else
        {
            c.get()->replace_node_in_sub_graph(old_child, new_child);
        }
        i++;
    }
}


// print graph in xml format
void Node_bt::print_xml_to(std::stringstream& file_stream, int depth)
{
    this->indent(file_stream, depth);

    // file_stream << "<" << this->node_name;
    file_stream << "<" << this->definition.xml_type;
    for(const xml_attribute& attr : this->definition.attributes)
    {
        file_stream << " " << attr.name << "=\"" << attr.value << "\"";
    }
    file_stream << ">\n";

    for(const std::shared_ptr<Node_bt>& child : this->children)
    {
        child->print_xml_to(file_stream, depth + 1);
    }

    this->indent(file_stream, depth);
    // file_stream << "</" << this->node_name << ">\n";
    file_stream << "</" << this->definition.xml_type << ">\n";
}

void Node_bt::recursive_append_node_names(std::set<std::string>& name_set, std::vector<std::string>& names_ordered)
{
    // append name
    name_set.insert(this->node_name);
    names_ordered.push_back(this->node_name);

    // call on all children nodes
    for(const std::shared_ptr<Node_bt>& c : this->children)
    {
        c.get()->recursive_append_node_names(name_set, names_ordered);
    }
}

std::vector<std::string> Node_bt::get_unique_node_names()
{
    std::set<std::string> name_set; // unique names in the graph
    std::vector<std::string> names_ordered; // all names in the graph, added in order
    std::vector<std::string> unique_names_ordered; // TODO, could be initialized with the correct size (same size as the set of unique names)


    // append this name
    name_set.insert(this->node_name);
    names_ordered.push_back(this->node_name);
    // call on all children nodes
    for(const std::shared_ptr<Node_bt>& c : this->children)
    {
        c.get()->recursive_append_node_names(name_set, names_ordered);
    }

    // find unique names in the order they where added
    std::set<std::string> tmp_name_set;
    for(const std::string& name : names_ordered)
    {
        if(tmp_name_set.count(name) == 0) // name does not exist yet
        {
            unique_names_ordered.push_back(name);
            tmp_name_set.insert(name);
        }
        else
        {
            continue;
        }
    }

    return unique_names_ordered;
}


void Node_bt::apply_attributes_recursive(const std::map<std::string, behavior_definition>& behavior_definitions)
{
    if(behavior_definitions.count(this->node_name) > 0)
    {
        this->definition = behavior_definitions.at(this->node_name);
    }
    else
    {
        std::cout << "Behavior definition is missing for Node " << this->node_name << " \n"; 
    }
    for(const auto& c : this->children)
    {
        c.get()->apply_attributes_recursive(behavior_definitions);
    }
}




///////////////////////////////// functions ////////////////////////////////////////

/*
    (old_parent)              (old_parent) 
        |                         |
        |                         |
    old_node      -->        (custom node)
                                |        |
                                |          |
                        (nodes_to_add) (old_node)
*/
// returns a reference to the new custom node
std::shared_ptr<Node_bt> add_as_children_to_custom_node(const std::vector<std::shared_ptr<Node_bt>>& nodes_to_add, const std::shared_ptr<Node_bt>& old_node, const std::shared_ptr<Node_bt>& custom_node, bool order_new_old)
{
    //Create custom pseudo parent node
    // std::shared_ptr<Node_bt> new_parent = std::make_shared<Node_bt>(action_custom_node);

    // If parent exist: remove this node from its parent and add the sequence node (with children) instead
    if(std::shared_ptr<Node_bt> p = old_node.get()->get_parent().lock())
    {
        // std::cout << "parent exists. name: " << p.get()->get_action_name() << "\n";
        p.get()->replace_child(old_node, custom_node);
        custom_node.get()->set_parent(p);
    }
    else
    {
        // std::cout << "Parent does not exist, cannot replace parent.\n";
    }

    if(order_new_old == true)
    {
        // add this and node_to_add as children
        for(const std::shared_ptr<Node_bt>& node_to_add : nodes_to_add)
        {
            custom_node.get()->add_child(node_to_add);
        }            
        custom_node.get()->add_child(old_node);
    }
    else
    {
        custom_node.get()->add_child(old_node);
        // add this and node_to_add as children
        for(const std::shared_ptr<Node_bt>& node_to_add : nodes_to_add)
        {
            custom_node.get()->add_child(node_to_add);
        }            
    }
    return custom_node;
}

// replaces this node, adds a sequence node with this node and node_to_add as children
// returns a reference to the sequence node
std::shared_ptr<Node_bt> add_with_sequence(const std::vector<std::shared_ptr<Node_bt>>& nodes_to_add, const std::shared_ptr<Node_bt>& old_node, bool order_new_old)
{
    std::shared_ptr<Node_bt> sequence_node = std::make_shared<Node_bt>("sequence", "ReactiveSequence");

    return add_as_children_to_custom_node(nodes_to_add, old_node, sequence_node, order_new_old); // return the sequence node
}

// replaces this node, adds a sequence node with this node and node_to_add as children
// returns a reference to the fallback node
std::shared_ptr<Node_bt> add_with_fallback(const std::vector<std::shared_ptr<Node_bt>>& nodes_to_add, const std::shared_ptr<Node_bt>& old_node, bool order_new_old)
{
    std::shared_ptr<Node_bt> fallback_node = std::make_shared<Node_bt>("fallback", "ReactiveFallback");

    return add_as_children_to_custom_node(nodes_to_add, old_node, fallback_node, order_new_old); // return the fallback node
}

std::shared_ptr<Node_bt> add_with_fallback(const std::shared_ptr<Node_bt>& node_to_add, const std::shared_ptr<Node_bt>& old_node, bool order_new_old)
{
    std::vector<std::shared_ptr<Node_bt>> nodes_to_add;
    nodes_to_add.push_back(node_to_add);
    return add_with_fallback(nodes_to_add, old_node, order_new_old);
}
std::shared_ptr<Node_bt> add_with_sequence(const std::shared_ptr<Node_bt>& node_to_add, const std::shared_ptr<Node_bt>& old_node, bool order_new_old)
{
    std::vector<std::shared_ptr<Node_bt>> nodes_to_add;
    nodes_to_add.push_back(node_to_add);
    return add_with_sequence(nodes_to_add, old_node, order_new_old);
}











 


std::string find_action_that_satisfies_condition(const std::vector<action>& action_tuples, const std::string& target_condition)
{
    for(const action& action : action_tuples)
    {
        for(const std::string& post_condition : action.post_conditions)
        {
            if(post_condition == target_condition)
            {
                // an action that solves target condition has been found
                return action.name;
            }
        }
    }

    std::cout << "ERROR! No action exists with the following post condition: " << target_condition << "\n";
    return ""; // TODO, return optional?
}


action get_action_by_name(const std::vector<action>& action_tuples, const std::string action_name)
{
    for(const action& action : action_tuples)
    {
        if(action.name == action_name)
        {
            return action;
        }
    }
    std::cout << "ERROR! No action exists with the following name: " << action_name << "\n";
    return {}; // TODO, return optional?
}





// returns the root node of a back-chained tree for the target condition
std::shared_ptr<Node_bt> back_chain_tree(const std::vector<action>& action_tuples, const std::string& target_condition)
{
    std::shared_ptr<Node_bt> root_node = std::make_shared<Node_bt>();

    std::vector<std::shared_ptr<Node_bt>> conditions_to_expand;

    // add start condition
    {
        action action;
        std::shared_ptr<Node_bt> start_condition = std::make_shared<Node_bt>(target_condition, "Action");
        conditions_to_expand.push_back(start_condition);
        root_node.get()->add_child(start_condition);
    }


    
    int i = 0;
    // while (conditions_to_expand.size() > 0)
    while (conditions_to_expand.empty() != true)
    {
        // get a condition to expand:
        std::shared_ptr<Node_bt> condition_to_expand = conditions_to_expand[0]; // expand first condition 

        std::cout << "expanding condition " << i++ << ": " << condition_to_expand.get()->get_node_name() << "\n";

        // find correct action for the 'condition to expand'
        std::string post_action_name = find_action_that_satisfies_condition(action_tuples, condition_to_expand.get()->get_node_name());
        action post_action = get_action_by_name(action_tuples, post_action_name);
        std::shared_ptr<Node_bt> post_action_node = std::make_shared<Node_bt>(post_action.name, "Action");

        // add all pre conditions as conditions to potentially be expanded
        std::vector<std::shared_ptr<Node_bt>> pre_condition_nodes;
        for(const auto& condition_name : post_action.pre_conditions)
        {
            // std::cout << "action: " << post_action.name << ". with pre condition: " << condition_name << "\n";
            std::shared_ptr<Node_bt> pre_condition = std::make_shared<Node_bt>(condition_name, "Action");
            // std::shared_ptr<Node_bt> pre_condition = std::make_shared<Node_bt>(find_action_by_name(action_tuples, condition_name));
            pre_condition_nodes.push_back(pre_condition);
            conditions_to_expand.push_back(pre_condition);
        }

        // Special case, sequence node is not needed if there are no pre conditions for that action
        std::shared_ptr<Node_bt> sequence_node;
        if(pre_condition_nodes.size() == 0)
        {
            sequence_node = post_action_node;
        }
        else
        {
            sequence_node = add_with_sequence(pre_condition_nodes, post_action_node, true);
        }
        
        std::shared_ptr<Node_bt> fallback_node = add_with_fallback(condition_to_expand, sequence_node, true);
        root_node.get()->replace_node_in_sub_graph(condition_to_expand, fallback_node);



        // delete first element (condition already exanded)
        conditions_to_expand.erase(conditions_to_expand.begin()); // not the most efficient implementation, but convenient for now. TODO: maybe use another list structure to reduce resizing of a vector...
    }

    return root_node.get()->get_children()[0]; // Remove the original root node. Only return the first fallback action of the back-chained tree
}

std::vector<std::string> get_unique_ordered_safety_conditions(const std::vector<action>& action_tuples, const std::shared_ptr<Node_bt>& task_tree)
{
    std::vector<std::string> safety_conditions;
    std::set<std::string> unordered_safety_conditions; 

    std::vector<std::string> unique_names = task_tree.get()->get_unique_node_names();

    for(const std::string& name : unique_names)
    {
        // find safety conditions of every action
        action a = find_action_by_name(action_tuples, name);

        for (const std::string& safety_condition : a.safety_conditions)
        {
            
            // check if safety condition is already added
            if(unordered_safety_conditions.count(safety_condition) == 0) // condition does not exist yet
            {
                safety_conditions.push_back(safety_condition);
                unordered_safety_conditions.insert(safety_condition);
            }
            else
            {
                continue;
            }
        }
    }

    // std::cout << "Unique nodes: \n"; 
    // for(const std::string& node_name : unique_names)
    // {
    //     std::cout << "\tName: " << node_name << "\n";
    // }
    // std::cout << "Safety conditions: \n"; 
    // for(const std::string& condition_name : safety_conditions)
    // {
    //     std::cout << "\tCondition: " << condition_name << "\n";
    // }

    return safety_conditions;
}



std::shared_ptr<Node_bt> generate_safety_tree(const std::vector<action>& action_tuples, const std::map<std::string, safety_condition>& safety_tuples, const std::shared_ptr<Node_bt>& task_tree)
{
    // Find all unique safety conditions (based on the task tree)
    std::vector<std::string> safety_conditions = get_unique_ordered_safety_conditions(action_tuples, task_tree);

    if(safety_conditions.size() == 0)
    {
        // no safety conditions to expand, return only the original task tree
        std::cout << "No safety conditions present, returning only task tree\n";
        return task_tree;
    }



    std::shared_ptr<Node_bt> safety_tree = std::make_shared<Node_bt>("sequence", "ReactiveSequence");
    for(const std::string& safety_condition : safety_conditions)
    {
        std::cout << "Expanding safety condition: " << safety_condition << "\n";
        // std::shared_ptr<Node_bt> condition_node = std::make_shared<Node_bt>(find_action_by_name(action_tuples, safety_condition));
        std::shared_ptr<Node_bt> condition_node = std::make_shared<Node_bt>(safety_condition, "Action");
        std::shared_ptr<Node_bt> sequence_node = std::make_shared<Node_bt>("sequence", "ReactiveSequence");
        std::shared_ptr<Node_bt> fallback_node = add_with_fallback(condition_node, sequence_node);


        std::vector<std::string> fallback_conditions_to_expand = safety_tuples.at(safety_condition).fallback_conditions; //{"at_goal"}; // TODO: Find all fallback conditions
        std::string fallback_action = safety_tuples.at(safety_condition).fallback_action;  // TODO: Find fallback action

        for(const std::string& fallback_condition : fallback_conditions_to_expand)
        {
            std::shared_ptr<Node_bt> expanded_safety_fallback_condition = back_chain_tree(action_tuples, fallback_condition);
            sequence_node.get()->add_child(expanded_safety_fallback_condition);
        }
        // sequence_node.get()->add_child(std::make_shared<Node_bt>(find_action_by_name(action_tuples, fallback_action)));
        sequence_node.get()->add_child(std::make_shared<Node_bt>(fallback_action, "Action"));

        safety_tree.get()->add_child(fallback_node);
    }

    std::shared_ptr<Node_bt> root_node = add_with_sequence(safety_tree, task_tree); // the expanded tree has a sequence node as root

    /*
        for all safety conditions:
            add safety conditions a sequence
            expand safety condition with fallback conditions and fallback action
            expand all fallback conditions using the back chaining principle

        add 'safety tree' and 'task tree' in a sequence
    */

    return root_node;
}







// TODO, return optional
action find_action_by_name(const std::vector<action>& action_tuples, const std::string& action_name)
{
    action a;

    // search through action tuples until a matching one is found
    for(const action& potential_action : action_tuples)
    {
        if(potential_action.name == action_name)
        {
            a = potential_action;
            break;
        }
    }


    return a;
}





} // end namespace bt_generation





