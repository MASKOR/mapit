#ifndef __DELETE_TEST_H
#define __DELETE_TEST_H

#include <QTest>
#include "../repositorycommon.h"

class DeleteTest : public RepositoryCommon
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void test_delete_entity_data();
    void test_delete_entity();
    void test_delete_tree_data();
    void test_delete_tree();
    void test_delete_sub_entity_data();
    void test_delete_sub_entity();
    void test_delete_sub_tree_data();
    void test_delete_sub_tree();

    void test_delete_entities_and_trees_mixed_data();
    void test_delete_entities_and_trees_mixed();
private:
    void add_bunny(std::shared_ptr<upns::Checkout> checkout, std::string path);
};

#endif
